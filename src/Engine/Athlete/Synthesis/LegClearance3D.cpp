#include "Engine/Athlete/Synthesis/LegClearance3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

// Os pontos mais perto entre os segmentos p1-q1 e p2-q2 (s: onde no
// primeiro, 0..1).
void closestSegments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3& c1, Vec3& c2, float& s) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float t = 0.0f;
    s = 0.0f;
    if (a <= 1e-9f && e <= 1e-9f) {
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 1e-9f) {
        t = std::clamp(f / e, 0.0f, 1.0f);
    } else {
        const float c = dot(d1, r);
        if (e <= 1e-9f) {
            s = std::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = dot(d1, d2), denominator = a * e - b * b;
            s = denominator > 1e-9f ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

} // namespace

void LegClearance3D::reset(const RagdollProfile3D& profile) {
    m_legs = resolveAthleteLegs3D(profile);
    m_geometry = resolveAthleteFootGeometry3D(profile, m_legs, 0.08f);
    m_ready = m_legs.valid && m_geometry.valid;
    m_applied = {};
}

LegClearance3D::Piece LegClearance3D::piece(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose,
    std::size_t link) const {
    // A capsula do link (a caixa do pe vira a capsula no eixo maior dela).
    const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
    const Vec3 center = pose.linkPositions[link] + pose.linkOrientations[link].rotate(collider.localPosition);
    const Quaternion q = (pose.linkOrientations[link] * collider.localOrientation).normalized();
    const Vec3 axis = q.rotate({ 1.0f, 0.0f, 0.0f });
    if (collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 h = collider.boxHalfExtents;
        const float half = std::max(0.0f, h.x - std::min(h.y, h.z));
        return { center - axis * half, center + axis * half, std::min(h.y, h.z) };
    }
    const float half = ragdollCapsuleHalfSegment3D(collider);
    return { center - axis * half, center + axis * half,
        0.5f * (collider.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(collider)) };
}

Vec3 LegClearance3D::needed(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose, int moving,
    Vec3 forward, const LegClearanceSettings3D& settings) const {
    // A canela e o pe que andam fora das capsulas do outro, sempre para fora
    // (o lado dele): o lado nunca muda, entao o pe nunca salta de um lado ao
    // outro - e as pernas nunca cruzam por dentro uma da outra.
    const std::size_t m = static_cast<std::size_t>(moving), f = 1 - m;
    const Vec3 outward = Vec3 { -forward.y, forward.x, 0.0f } * (moving == 0 ? 1.0f : -1.0f);
    const Vec3 footOrigin = pose.linkPositions[m_legs.foot[m]];
    const Piece movingPieces[2] = { piece(profile, pose, m_legs.foot[m]), piece(profile, pose, m_legs.shin[m]) };
    const Piece fixedPieces[2] = { piece(profile, pose, m_legs.foot[f]), piece(profile, pose, m_legs.shin[f]) };
    float push = 0.0f;
    for (int i = 0; i < 2; ++i) {
        const Piece& a = movingPieces[i];
        for (const Piece& b : fixedPieces) {
            Vec3 c1, c2;
            float s = 0.0f;
            closestSegments(a.a, a.b, b.a, b.b, c1, c2, s);
            const float reach = a.radius + b.radius + settings.margin;
            const Vec3 gap = c1 - c2;
            if (gap.lengthSquared() >= reach * reach) continue;
            // O que falta para fora: o resto da distancia (a frente e no alto)
            // ja conta.
            const float lateral = dot(gap, outward);
            const Vec3 rest = gap - outward * lateral;
            const float required = std::sqrt(std::max(0.0f, reach * reach - rest.lengthSquared()));
            if (lateral >= required) continue;
            // (A canela: o tornozelo andando leva o ponto do toque so a parte
            // dele a partir do joelho - perto do joelho, empurrar o pe nao
            // adianta e o empurrao estourava; joelho com joelho fica.)
            float lever = 1.0f;
            if (i == 1) {
                const bool ankleAtB = (a.b - footOrigin).lengthSquared() < (a.a - footOrigin).lengthSquared();
                lever = ankleAtB ? s : 1.0f - s;
                if (lever < settings.shinLeverMinimum) continue;
            }
            push = std::max(push, (required - lateral) / lever);
        }
    }
    return outward * std::min(push, settings.maximumPush);
}

void LegClearance3D::apply(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, Vec3 rootPosition,
    Quaternion rootOrientation, RagdollAnimationPose3D& pose, std::array<bool, 2> swinging, float deltaTime,
    const LegClearanceSettings3D& settings) {
    if (!m_ready || coordinates.size() != profile.links.size() || pose.linkPositions.size() != profile.links.size())
        return;
    const float dt = std::max(deltaTime, 1e-4f);
    // Quem contorna: o pe no balanco (os dois, o que ja contornava, senao o
    // mais alto - a sola).
    int moving = swinging[0] == swinging[1] ? -1 : (swinging[0] ? 0 : 1);
    if (swinging[0] && swinging[1] && m_lastMoving >= 0) {
        moving = m_lastMoving;
    } else if (swinging[0] && swinging[1]) {
        std::array<float, 2> sole {};
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t link = m_legs.foot[side];
            const Vec3 heel = pose.linkPositions[link] + pose.linkOrientations[link].rotate(m_geometry.heel[side]);
            const Vec3 toe = pose.linkPositions[link] + pose.linkOrientations[link].rotate(m_geometry.toeBase[side]);
            sole[side] = std::min(heel.z, toe.z);
        }
        moving = sole[0] > sole[1] ? 0 : 1;
    }
    m_lastMoving = moving;
    const Vec3 forward = [&] {
        const Vec3 f = planar((rootOrientation * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f }));
        return f.lengthSquared() > 1e-6f ? f.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    }();
    const Vec3 left { -forward.y, forward.x, 0.0f };
    // Soltando: numa mola; apoiado, bem devagar (parado de vez, os dois pes
    // ficavam com empurroes velhos e brigavam na troca seguinte).
    const auto release = [&](std::size_t side, Vec3 want) {
        Vec3& applied = m_applied[side];
        Vec3 change = (want - applied) * std::min(1.0f, dt / std::max(0.005f, settings.releaseSeconds));
        const float most = (swinging[side] ? settings.maximumReleaseSpeed : settings.plantedReleaseSpeed) * dt;
        if (change.length() > most) change = change * (most / change.length());
        applied += change;
    };
    const auto push = [&](int side, Vec3 offset) {
        Vec3 ankle, knee;
        Quaternion foot;
        athleteLegState3D(profile, m_legs, side, coordinates, rootPosition, rootOrientation, ankle, knee, foot);
        solveAthleteLeg3D(profile, m_legs, side, coordinates, rootPosition, rootOrientation, ankle + offset, knee, foot);
        athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, pose);
    };
    // 1) O outro pe primeiro: o empurrao que ainda tem (soltando no balanco,
    // parado no apoio) - o que contorna faz a conta contra onde ele esta.
    for (int side = 0; side < 2; ++side) {
        if (side == moving) continue;
        const std::size_t s = static_cast<std::size_t>(side);
        release(s, {});
        if (m_applied[s].lengthSquared() > 1e-8f) push(side, m_applied[s]);
    }
    if (moving < 0) return;
    // 2) O que contorna: precisando de mais, entra numa mola rapida, sem
    // deixar mais que a tolerancia de toque; soltando, a mola mais lenta.
    const std::size_t s = static_cast<std::size_t>(moving);
    Vec3& applied = m_applied[s];
    const Vec3 want = needed(profile, pose, moving, forward, settings);
    if (want.lengthSquared() > applied.lengthSquared() && dot(want, applied) >= 0.0f) {
        // (A mola, deixando no maximo a tolerancia de toque - mas nunca mais
        // depressa que maximumPushSpeed: um toque de um instante e melhor que
        // o pe saltando de lado.)
        Vec3 change = (want - applied) * std::min(1.0f, dt / std::max(0.005f, settings.pushSeconds));
        const Vec3 short_ = want - applied - change;
        const float missing = short_.length();
        if (missing > settings.touchTolerance) change += short_ * ((missing - settings.touchTolerance) / missing);
        const float most = settings.maximumPushSpeed * dt;
        if (change.length() > most) change = change * (most / change.length());
        applied += change;
    } else {
        release(s, want);
    }
    if (applied.lengthSquared() < 1e-8f) return;
    push(moving, applied);
    // 3) O que o joelho mexido ainda deixou (alem da tolerancia).
    Vec3 more = needed(profile, pose, moving, forward, settings);
    const float extraNeed = more.length();
    if (extraNeed <= settings.touchTolerance) return;
    more = more * ((extraNeed - settings.touchTolerance) / extraNeed);
    Vec3 total = applied + more;
    if (total.length() > settings.maximumPush) total = total * (settings.maximumPush / total.length());
    const Vec3 extra = total - applied;
    applied = total;
    push(moving, extra);
}

} // namespace MatterEngine
