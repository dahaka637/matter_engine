#include "Engine/Athlete/Body/ActiveStance3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float minimumJerk(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return s * s * s * (10.0f - 15.0f * s + 6.0f * s * s);
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
    m_step = {};
    m_stepsTaken = 0;
    m_outsideTicks = 0;
    m_relax = 0.0f;
    m_mismatch = 0.0f;
    // (Entra ancorada: os pes estao no chao e a pose-alvo parte da do corpo;
    // andando, solta na mistura.)
    m_baseWeight = 1.0f;
    m_baseTarget = {};
    m_baseShift = {};
    m_baseBias = {};
    m_baseMask = 0;
    m_baseFresh = true;
}

void ActiveStance3D::anchor(int side, const RagdollProfile3D& profile, const RagdollState3D& body,
    const FootObservation3D& seen) {
    // Preso plano no chao, no rumo em que esta (um pe que caiu de lado nao
    // entorta a perna e o corpo junto).
    Foot& foot = m_feet[static_cast<std::size_t>(side)];
    const std::size_t footLink = m_legs.foot[static_cast<std::size_t>(side)];
    const RagdollLinkDefinition3D& definition = profile.links[footLink];
    const float yaw = std::atan2(seen.forward.y, seen.forward.x);
    foot.anchored = true;
    foot.orientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw) * definition.modelOrientation).normalized();
    foot.position = body.links[footLink].position;
    foot.position.z = seen.soleHeight + m_soleToOrigin[static_cast<std::size_t>(side)];
    foot.release = 0.0f;
}

Vec3 ActiveStance3D::stepTarget(int side, Vec3 capture, Vec3 left, const ActiveStanceSettings3D& settings) const {
    // Do pe que fica ate alem da captura (no plano), no alcance, sem cruzar
    // a linha dele: o pe esquerdo fica pelo menos stepSideGap a esquerda do
    // direito, no rumo da pelve (a captura do outro lado e do proximo passo).
    const Foot& stance = m_feet[static_cast<std::size_t>(1 - side)];
    Vec3 away { capture.x - stance.position.x, capture.y - stance.position.y, 0.0f };
    float reach = away.length();
    const Vec3 direction = reach > 1e-4f ? away * (1.0f / reach) : left * (side == 0 ? 1.0f : -1.0f);
    reach = std::clamp(reach + settings.stepOvershoot, settings.stepMinimumWidth, settings.stepMaximumReach);
    Vec3 offset = direction * reach;
    const float sign = side == 0 ? 1.0f : -1.0f;
    const float lateral = offset.x * left.x + offset.y * left.y;
    if (lateral * sign < settings.stepSideGap) offset += left * (sign * settings.stepSideGap - lateral);
    return { stance.position.x + offset.x, stance.position.y + offset.y, 0.0f };
}

void ActiveStance3D::adapt(const RagdollProfile3D& profile, const RagdollState3D& body,
    const BodyObservation3D& observation, std::array<bool, 2> clipPlanted, float targetSpeed,
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

    // Os pes: presos onde tocam o chao (o clipe diz apoio), soltos no balanco.
    for (int side = 0; side < 2; ++side) {
        if (side == m_step.side) continue;
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        const FootObservation3D& seen = observation.feet[static_cast<std::size_t>(side)];
        if (clipPlanted[side]) {
            if (!foot.anchored && seen.supporting) anchor(side, profile, body, seen);
        } else if (foot.anchored) {
            foot.anchored = false;
            foot.release = 1.0f;
            foot.releasePosition = foot.position;
            foot.releaseOrientation = foot.orientation;
        }
    }

    // Passo de equilibrio: a captura saiu da base. Para cada pe, a base nova
    // (o que fica mais o lugar do pouso, sem cruzar): sai o que deixa a
    // captura mais para dentro dela (o caminho do pe desempata); o outro
    // precisa estar preso e apoiado.
    const Vec3 left = [&] {
        const Vec3 axis = rootOrientation.rotate({ 0.0f, 1.0f, 0.0f });
        const Vec3 flat { axis.x, axis.y, 0.0f };
        return flat.lengthSquared() > 1e-6f ? flat.normalized() : Vec3 { 0.0f, 1.0f, 0.0f };
    }();
    m_outsideTicks = m_captureOutside > settings.stepMargin ? m_outsideTicks + 1 : 0;
    const auto soleCorners = [](std::vector<std::array<float, 2>>& corners, Vec3 center, Vec3 forward) {
        const Vec3 lateral = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward);
        for (const float along : { -0.11f, 0.13f })
            for (const float across : { -0.045f, 0.045f }) {
                const Vec3 corner = center + forward * along + lateral * across;
                corners.push_back({ corner.x, corner.y });
            }
    };
    if (m_step.side < 0 && settings.balanceSteps && m_outsideTicks >= settings.stepTriggerTicks
        && m_stepsTaken < settings.maximumSteps) {
        int swing = -1;
        float cheapest = 1e9f;
        for (int side = 0; side < 2; ++side) {
            const int stance = 1 - side;
            const FootObservation3D& stanceSeen = observation.feet[static_cast<std::size_t>(stance)];
            if (!m_feet[static_cast<std::size_t>(stance)].anchored || !stanceSeen.supporting) continue;
            const Vec3 now = body.links[m_legs.foot[static_cast<std::size_t>(side)]].position;
            const Vec3 target = stepTarget(side, capture, left, settings);
            std::vector<std::array<float, 2>> corners;
            soleCorners(corners, stanceSeen.soleCenter, stanceSeen.forward);
            soleCorners(corners, target, observation.feet[static_cast<std::size_t>(side)].forward);
            const float cost = outsideHull(corners, capture.x, capture.y) * settings.stepCatchWeight
                + planarDistance(now, target);
            if (cost < cheapest) {
                cheapest = cost;
                swing = side;
            }
        }
        if (swing >= 0) {
            const std::size_t swingLink = m_legs.foot[static_cast<std::size_t>(swing)];
            Foot& foot = m_feet[static_cast<std::size_t>(swing)];
            m_step = {};
            m_step.side = swing;
            m_step.from = body.links[swingLink].position;
            m_step.target = stepTarget(swing, capture, left, settings);
            m_step.orientation = foot.anchored ? foot.orientation : body.links[swingLink].orientation;
            m_step.ground = m_feet[static_cast<std::size_t>(1 - swing)].position.z;
            foot.anchored = false;
            foot.release = 0.0f;
        }
    }
    if (m_step.side >= 0) {
        // O balanco: o alvo segue a captura; o pe pousa no contato.
        m_step.time += dt;
        const float s = m_step.time / std::max(0.05f, settings.stepSeconds);
        const Vec3 wanted = stepTarget(m_step.side, capture, left, settings);
        m_step.target += (wanted - m_step.target) * std::min(1.0f, dt / 0.05f);
        // No passo a base e o pe que fica mais o lugar do pouso: e contra ela
        // que a captura conta (com um pe so, ela esta sempre fora).
        {
            std::vector<std::array<float, 2>> corners;
            const FootObservation3D& stance = observation.feet[static_cast<std::size_t>(1 - m_step.side)];
            if (stance.supporting) soleCorners(corners, stance.soleCenter, stance.forward);
            soleCorners(corners, m_step.target, observation.feet[static_cast<std::size_t>(m_step.side)].forward);
            m_captureOutside = outsideHull(corners, capture.x, capture.y);
        }
        const FootObservation3D& seen = observation.feet[static_cast<std::size_t>(m_step.side)];
        if ((s >= settings.stepTouchShare && seen.supporting) || s >= settings.stepTimeoutShare) {
            if (seen.supporting) anchor(m_step.side, profile, body, seen);
            m_step = {};
            ++m_stepsTaken;
        }
    }
    if (m_step.side < 0 && m_captureOutside <= 0.0f && m_relax <= 0.0f) m_stepsTaken = 0;

    // A raiz sobre a base de verdade (filtrada); no passo, a raiz vai ate a
    // pelve de verdade (o passo equilibra; o servo nao briga com a queda).
    {
        // O deslocamento acompanha o clipe na hora (e so a pose dele em
        // relacao aos pes); quando muda o conjunto de pes presos (um sai,
        // outro pousa), o salto vira um vies que some em baseFilterSeconds.
        Vec3 shift;
        int count = 0;
        int mask = 0;
        for (int side = 0; side < 2; ++side) {
            const Foot& foot = m_feet[static_cast<std::size_t>(side)];
            if (side == m_step.side || !foot.anchored) continue;
            shift += Vec3 { foot.position.x - clipFoot[side].x, foot.position.y - clipFoot[side].y, 0.0f };
            ++count;
            mask |= 1 << side;
        }
        const bool standing = settings.anchorRootToBase && count > 0 && targetSpeed < settings.stillSpeed
            && clipPlanted[0] && clipPlanted[1];
        const float rate = dt / std::max(0.02f, settings.baseAnchorSeconds);
        m_baseWeight = standing ? std::min(1.0f, m_baseWeight + rate) : std::max(0.0f, m_baseWeight - rate);
        if (count > 0) {
            const Vec3 raw = shift * (1.0f / static_cast<float>(count));
            if (m_baseFresh) m_baseBias = {};
            else if (mask != m_baseMask) m_baseBias += m_baseTarget - raw;
            m_baseBias *= 1.0f - std::min(1.0f, dt / std::max(dt, settings.baseFilterSeconds));
            m_baseTarget = raw + m_baseBias;
            m_baseMask = mask;
            m_baseFresh = false;
        }
        m_baseShift = m_baseTarget * smooth(m_baseWeight);
        rootPosition += m_baseShift;
        m_relax = m_step.side >= 0 ? settings.stepRelax
            : std::max(0.0f, m_relax - dt * settings.stepRelax / std::max(0.05f, settings.stepRelaxSeconds));
        if (m_relax > 0.0f) {
            const Vec3 pelvis = body.links.front().position;
            rootPosition.x += (pelvis.x - rootPosition.x) * m_relax;
            rootPosition.y += (pelvis.y - rootPosition.y) * m_relax;
        }
    }

    float mismatch = 0.0f;
    for (int side = 0; side < 2; ++side) {
        Foot& foot = m_feet[static_cast<std::size_t>(side)];
        Vec3 targetPosition = clipFoot[side] + m_baseShift;
        Quaternion targetOrientation = clipOrientation[side];
        bool solve = m_baseWeight > 0.0f;
        bool fromPelvis = false;
        if (side == m_step.side) {
            // O pe do passo: jerk minimo ate o alvo, no arco.
            const float s = std::clamp(m_step.time / std::max(0.05f, settings.stepSeconds), 0.0f, 1.0f);
            const float u = minimumJerk(s);
            // (No fim, um pouco abaixo do chao: o motor aperta o pe nele.)
            const float ground = m_step.ground - settings.stepPress;
            targetPosition = { m_step.from.x + (m_step.target.x - m_step.from.x) * u,
                m_step.from.y + (m_step.target.y - m_step.from.y) * u,
                m_step.from.z + (ground - m_step.from.z) * u + settings.stepHeight * std::sin(3.14159265f * s) };
            targetOrientation = m_step.orientation;
            solve = true;
            fromPelvis = true;
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
        if (fromPelvis) {
            // O passo sai da pelve de verdade: com o corpo inclinado, os
            // angulos tirados da pelve do plano poriam o pe longe de onde ele
            // precisa pousar no mundo.
            solveAthleteLeg3D(profile, m_legs, side, coordinates, body.links.front().position,
                (body.links.front().orientation * profile.links.front().modelOrientation.conjugate()).normalized(),
                ankle, clipKnee[side], targetOrientation);
        } else {
            solveAthleteLeg3D(profile, m_legs, side, coordinates, rootPosition, rootOrientation, ankle, clipKnee[side],
                targetOrientation);
        }
    }
    if (m_step.side >= 0) mismatch = std::max(mismatch, 0.1f);
    m_mismatch = mismatch;
    athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
}

} // namespace MatterEngine
