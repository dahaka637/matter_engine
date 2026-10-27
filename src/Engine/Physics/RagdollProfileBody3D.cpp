#include "Engine/Physics/RagdollProfileBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace MatterEngine {

namespace {

// A regiao que continua a cadeia (o filho cujo eixo o link segue).
std::optional<PhysiqueRegion3D> continuation(PhysiqueRegion3D region) {
    switch (region) {
    case PhysiqueRegion3D::Pelvis: return PhysiqueRegion3D::Abdomen;
    case PhysiqueRegion3D::Abdomen: return PhysiqueRegion3D::Chest;
    case PhysiqueRegion3D::Chest: return PhysiqueRegion3D::UpperChest;
    case PhysiqueRegion3D::UpperChest: return PhysiqueRegion3D::Neck;
    case PhysiqueRegion3D::Neck: return PhysiqueRegion3D::Head;
    case PhysiqueRegion3D::UpperArm: return PhysiqueRegion3D::Forearm;
    case PhysiqueRegion3D::Forearm: return PhysiqueRegion3D::Hand;
    case PhysiqueRegion3D::Thigh: return PhysiqueRegion3D::Shin;
    case PhysiqueRegion3D::Shin: return PhysiqueRegion3D::Foot;
    default: return std::nullopt;
    }
}

Vec3 colliderAxis(const RagdollLinkDefinition3D& link) {
    return (link.modelOrientation * link.collider.localOrientation).normalized().rotate({ 1.0f, 0.0f, 0.0f });
}

// O ponto levado pelo campo do tonus: a parte em volta do eixo vezes a
// grossura, a parte ao longo dele igual.
Vec3 girthMap(const BodyGirthAxis3D& axis, float girth, Vec3 point) {
    const Vec3 offset = point - axis.point;
    const Vec3 along = axis.direction * dot(offset, axis.direction);
    return axis.point + along + (offset - along) * girth;
}

struct Segment {
    Vec3 from, to;
    float radiusFrom = 0.0f, radiusTo = 0.0f;
};

Segment posedCapsule(const RagdollLinkDefinition3D& link, Vec3 position, Quaternion orientation) {
    const RagdollCapsuleDefinition3D& capsule = link.collider;
    const Vec3 center = position + orientation.rotate(capsule.localPosition);
    const Vec3 axis = (orientation * capsule.localOrientation).normalized().rotate({ 1.0f, 0.0f, 0.0f });
    const float half = ragdollCapsuleHalfSegment3D(capsule);
    return { center - axis * half, center + axis * half, capsule.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(capsule) };
}

// Os pontos mais perto entre dois segmentos (s e t de 0 a 1 em cada um).
void closestOnSegments(const Segment& a, const Segment& b, float& s, float& t) {
    const Vec3 d1 = a.to - a.from, d2 = b.to - b.from, r = a.from - b.from;
    const float aa = dot(d1, d1), ee = dot(d2, d2), f = dot(d2, r);
    constexpr float Tiny = 1e-12f;
    if (aa <= Tiny && ee <= Tiny) { s = t = 0.0f; return; }
    if (aa <= Tiny) { s = 0.0f; t = std::clamp(f / ee, 0.0f, 1.0f); return; }
    const float c = dot(d1, r);
    if (ee <= Tiny) { t = 0.0f; s = std::clamp(-c / aa, 0.0f, 1.0f); return; }
    const float bb = dot(d1, d2), denominator = aa * ee - bb * bb;
    s = denominator > Tiny ? std::clamp((bb * f - c * ee) / denominator, 0.0f, 1.0f) : 0.0f;
    t = (bb * s + f) / ee;
    if (t < 0.0f) { t = 0.0f; s = std::clamp(-c / aa, 0.0f, 1.0f); }
    else if (t > 1.0f) { t = 1.0f; s = std::clamp((bb - c) / aa, 0.0f, 1.0f); }
}

} // namespace

std::vector<BodyGirthAxis3D> bodyGirthAxes3D(const RagdollProfile3D& profile) {
    const std::size_t count = profile.links.size();
    std::vector<BodyGirthAxis3D> axes(count);
    std::vector<std::optional<PhysiqueRegion3D>> regions(count);
    for (std::size_t i = 0; i < count; ++i) regions[i] = physiqueRegionOfLink3D(profile.links[i].id);
    for (std::size_t i = 0; i < count; ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        BodyGirthAxis3D& axis = axes[i];
        axis.point = link.parentIndex >= 0 ? link.inboundJoint.anchorModelPosition : link.modelPosition;
        Vec3 direction;
        if (const std::optional<PhysiqueRegion3D> next = regions[i] ? continuation(*regions[i]) : std::nullopt) {
            for (std::size_t k = 0; k < count; ++k)
                if (profile.links[k].parentIndex == static_cast<int>(i) && regions[k] == next) {
                    direction = profile.links[k].inboundJoint.anchorModelPosition - axis.point;
                    axis.length = direction.length();
                    axis.child = static_cast<int>(k);
                }
        }
        if (direction.lengthSquared() < 1e-8f && regions[i] == PhysiqueRegion3D::Hand) {
            // A mao: o colisor dela, do punho para os dedos.
            direction = colliderAxis(link);
            if (dot(direction, link.modelPosition - axis.point) < 0.0f) direction = -direction;
        }
        if (direction.lengthSquared() < 1e-8f) direction = { 0.0f, 0.0f, 1.0f };
        axis.direction = direction.normalized();
    }
    return axes;
}

float bodyGirthAlong3D(const std::vector<BodyGirthAxis3D>& axes, const std::vector<float>& girth, std::size_t link, Vec3 point) {
    const BodyGirthAxis3D& axis = axes[link];
    if (axis.child < 0 || axis.length <= 1e-6f) return girth[link];
    const float t = dot(point - axis.point, axis.direction) / axis.length;
    const float u = std::clamp((t - 0.55f) / 0.45f, 0.0f, 1.0f);
    return girth[link] + (girth[static_cast<std::size_t>(axis.child)] - girth[link]) * u * u * (3.0f - 2.0f * u);
}

float ragdollRestCapsuleGap3D(const RagdollLinkDefinition3D& a, const RagdollLinkDefinition3D& b) {
    return ragdollPosedCapsuleGap3D(a, a.modelPosition, a.modelOrientation, b, b.modelPosition, b.modelOrientation);
}

float ragdollPosedCapsuleGap3D(const RagdollLinkDefinition3D& a, Vec3 positionA, Quaternion orientationA,
    const RagdollLinkDefinition3D& b, Vec3 positionB, Quaternion orientationB) {
    return ragdollPosedCapsuleContact3D(a, positionA, orientationA, b, positionB, orientationB).gap;
}

RagdollCapsuleContact3D ragdollPosedCapsuleContact3D(const RagdollLinkDefinition3D& a, Vec3 positionA, Quaternion orientationA,
    const RagdollLinkDefinition3D& b, Vec3 positionB, Quaternion orientationB) {
    const Segment first = posedCapsule(a, positionA, orientationA), second = posedCapsule(b, positionB, orientationB);
    float s = 0.0f, t = 0.0f;
    closestOnSegments(first, second, s, t);
    const Vec3 p = first.from + (first.to - first.from) * s, q = second.from + (second.to - second.from) * t;
    const float ra = first.radiusFrom + (first.radiusTo - first.radiusFrom) * s;
    const float rb = second.radiusFrom + (second.radiusTo - second.radiusFrom) * t;
    return { (p - q).length() - ra - rb, s, t };
}

RagdollProfile3D bodyRagdollProfile3D(const RagdollProfile3D& base, const PlayerBody3D& body) {
    if (isBaseBody3D(body)) return base;
    RagdollProfile3D result = base;
    const std::size_t count = base.links.size();
    const float s = bodyScale3D(body), tone = bodyTone3D(body);
    const BodyMotorScale3D motor = bodyMotorScale3D(body);
    const std::vector<BodyGirthAxis3D> axes = bodyGirthAxes3D(base);

    // A grossura de cada link (a chuteira e os links sem regiao nao mudam).
    std::vector<float> girth(count, 1.0f);
    for (std::size_t i = 0; i < count; ++i)
        if (const std::optional<PhysiqueRegion3D> region = physiqueRegionOfLink3D(base.links[i].id))
            girth[i] = std::max(0.5f, bodyGirth3D(*region, tone));

    // As massas: a de cada link na base mais a parte do ganho nominal do tonus
    // (o peso a 1,80 m da forma, pelo volume da malha) pelo volume que ele
    // ganhou (a grossura ao quadrado); depois a soma vai ao peso de verdade.
    std::vector<float> mass(count);
    float growth = 0.0f, baseTotal = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
        mass[i] = base.links[i].massFraction * base.totalMassKg;
        baseTotal += mass[i];
        growth += mass[i] * (girth[i] * girth[i] - 1.0f);
    }
    if (std::abs(growth) > 1e-6f) {
        const float nominalGain = BaseBodyMassKg * (bodyToneVolume3D(tone) - 1.0f);
        for (std::size_t i = 0; i < count; ++i)
            mass[i] += nominalGain * mass[i] * (girth[i] * girth[i] - 1.0f) / growth;
    }
    float total = 0.0f;
    for (float& m : mass) total += (m = std::max(m, 1e-4f * baseTotal));

    for (std::size_t i = 0; i < count; ++i) {
        RagdollLinkDefinition3D& link = result.links[i];
        const RagdollLinkDefinition3D& source = base.links[i];
        RagdollCapsuleDefinition3D& collider = link.collider;
        // A grossura em volta do eixo do link (no repouso, antes da escala).
        const Vec3 center = source.modelPosition + source.modelOrientation.rotate(source.collider.localPosition);
        const float centerGirth = bodyGirthAlong3D(axes, girth, i, center);
        if (collider.shape == RagdollColliderShape3D::Capsule && (girth[i] != 1.0f || centerGirth != 1.0f)) {
            const Vec3 moved = girthMap(axes[i], centerGirth, center);
            collider.localPosition = source.modelOrientation.conjugate().rotate(moved - source.modelPosition);
            // A extensao da capsula no eixo dela segue a da malha (como o
            // exportador mede): ao longo do osso fica (os raios maiores comem
            // o cilindro, ate o minimo de 2 mm dele); de lado (o tronco
            // deitado), cresce com a grossura. Cada ponta, a grossura dela.
            const Vec3 capsuleAxis = colliderAxis(source);
            const float along = dot(capsuleAxis, axes[i].direction);
            const float stretch = std::sqrt(along * along + centerGirth * centerGirth * (1.0f - along * along));
            const float half = ragdollCapsuleHalfSegment3D(source.collider);
            if (source.collider.radiusAtPositiveXMeters > 0.0f) {
                collider.radiusMeters = source.collider.radiusMeters * bodyGirthAlong3D(axes, girth, i, center - capsuleAxis * half);
                collider.radiusAtPositiveXMeters
                    = source.collider.radiusAtPositiveXMeters * bodyGirthAlong3D(axes, girth, i, center + capsuleAxis * half);
            } else {
                // O tronco deitado: o raio e a meia profundidade (a cintura do
                // franzino so afina dos lados; as costas do encorpado crescem
                // pouco, entao o centro vai para a frente - x e a frente). Ao
                // longo do osso (o pescoco), so o centro vai para a frente: o
                // raio redondo segue a largura.
                float depth = 1.0f, forward = 0.0f;
                if (const std::optional<PhysiqueRegion3D> region = physiqueRegionOfLink3D(source.id)) {
                    const float wide = std::max(1e-3f, bodyGirth3D(*region, tone));
                    const float front = bodyGirthDepth3D(*region, tone), back = bodyGirthBack3D(*region, tone);
                    if (std::abs(along) < 0.5f) depth = 0.5f * (front + back) / wide;
                    forward = 0.5f * (front - back) / wide;
                }
                collider.radiusMeters = source.collider.radiusMeters * centerGirth * depth;
                collider.localPosition += source.modelOrientation.conjugate().rotate(
                    Vec3 { source.collider.radiusMeters * centerGirth * forward, 0.0f, 0.0f });
            }
            // O tronco deitado e o nucleo (o exportador o estreita para o
            // braco solto nao encostar): a largura dele fica a de hoje, so a
            // profundidade cresce.
            const float radii = collider.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(collider);
            const float extent = std::abs(along) < 0.5f ? source.collider.lengthMeters : source.collider.lengthMeters * stretch;
            collider.lengthMeters = std::max(extent, radii + 0.002f);
        } else if (girth[i] != 1.0f && collider.shape == RagdollColliderShape3D::Box) {
            const Vec3 local = source.modelOrientation.conjugate().rotate(axes[i].direction);
            const auto stretch = [&](float component) { return std::sqrt(component * component + girth[i] * girth[i] * (1.0f - component * component)); };
            collider.boxHalfExtents = { collider.boxHalfExtents.x * stretch(local.x), collider.boxHalfExtents.y * stretch(local.y),
                collider.boxHalfExtents.z * stretch(local.z) };
        }
        // Os comprimentos x s.
        link.modelPosition = source.modelPosition * s;
        link.inboundJoint.anchorModelPosition = source.inboundJoint.anchorModelPosition * s;
        link.centerOfMassLocal = source.centerOfMassLocal * s;
        collider.localPosition = collider.localPosition * s;
        collider.radiusMeters *= s;
        if (collider.radiusAtPositiveXMeters > 0.0f) collider.radiusAtPositiveXMeters *= s;
        collider.lengthMeters *= s;
        collider.boxHalfExtents = collider.boxHalfExtents * s;
        if (collider.shape == RagdollColliderShape3D::ConvexHull) {
            for (Vec3& point : collider.hullPoints) point = point * s;
            prepareRagdollConvexHull3D(collider);
        }
        // A massa e os motores (a carga).
        link.massFraction = mass[i] / total;
        for (RagdollAxisDefinition3D& axis : link.inboundJoint.axes) {
            axis.stiffness *= motor.stiffness;
            axis.damping *= motor.damping;
            axis.maximumTorque *= motor.torque;
        }
    }
    result.totalMassKg = body.massKg;
    result.standingRootHeightMeters = base.standingRootHeightMeters * s;
    result.uniformRadiusMeters = base.uniformRadiusMeters * s;

    // As coxas afastadas no repouso (a folga do exportador, x s): so o raio de
    // cima (o da virilha) encolhe.
    int left = -1, right = -1;
    for (std::size_t i = 0; i < count; ++i) {
        if (result.links[i].id == "LeftThigh") left = static_cast<int>(i);
        if (result.links[i].id == "RightThigh") right = static_cast<int>(i);
    }
    if (left >= 0 && right >= 0) {
        constexpr float Clearance = 0.024f;
        RagdollLinkDefinition3D& a = result.links[static_cast<std::size_t>(left)];
        RagdollLinkDefinition3D& b = result.links[static_cast<std::size_t>(right)];
        const float wanted = std::min(Clearance * s, ragdollRestCapsuleGap3D(base.links[static_cast<std::size_t>(left)],
                                                         base.links[static_cast<std::size_t>(right)]) * s);
        for (int round = 0; round < 20; ++round) {
            const float gap = ragdollRestCapsuleGap3D(a, b);
            if (gap >= wanted - 1e-5f) break;
            const float shrink = 0.5f * (wanted - gap) + 0.0002f;
            for (RagdollLinkDefinition3D* thigh : { &a, &b }) {
                RagdollCapsuleDefinition3D& capsule = thigh->collider;
                // A ponta de cima: a mais perto da junta do quadril.
                const Vec3 center = thigh->modelPosition + thigh->modelOrientation.rotate(capsule.localPosition);
                const bool upperIsNegative = dot(colliderAxis(*thigh), thigh->inboundJoint.anchorModelPosition - center) < 0.0f;
                const float half = ragdollCapsuleHalfSegment3D(capsule);
                if (capsule.radiusAtPositiveXMeters <= 0.0f) capsule.radiusAtPositiveXMeters = capsule.radiusMeters;
                float& radius = upperIsNegative ? capsule.radiusMeters : capsule.radiusAtPositiveXMeters;
                radius = std::max(0.5f * radius, radius - shrink);
                capsule.lengthMeters = 2.0f * half + capsule.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(capsule);
            }
        }
    }
    return result;
}

} // namespace MatterEngine
