#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace MatterEngine {
namespace {

using Segment = std::pair<Vec3, Vec3>;

RagdollColliderWireframeSide3D sideFor(std::string_view linkId) {
    if (linkId.starts_with("Left")) return RagdollColliderWireframeSide3D::Left;
    if (linkId.starts_with("Right")) return RagdollColliderWireframeSide3D::Right;
    return RagdollColliderWireframeSide3D::Center;
}

Vec3 toModel(const RagdollLinkDefinition3D& link, Vec3 colliderLocal) {
    const Vec3 linkLocal = link.collider.localPosition + link.collider.localOrientation.rotate(colliderLocal);
    return link.modelPosition + link.modelOrientation.rotate(linkLocal);
}

void appendRing(
    std::vector<Segment>& segments, const RagdollLinkDefinition3D& link, float x, float radius, int radialSegments) {
    if (!(radius > 0.000001f)) return;
    for (int index = 0; index < radialSegments; ++index) {
        const float angle0 =
            2.0f * std::numbers::pi_v<float> * static_cast<float>(index) / static_cast<float>(radialSegments);
        const float angle1 =
            2.0f * std::numbers::pi_v<float> * static_cast<float>(index + 1) / static_cast<float>(radialSegments);
        segments.emplace_back(toModel(link, { x, radius * std::cos(angle0), radius * std::sin(angle0) }),
            toModel(link, { x, radius * std::cos(angle1), radius * std::sin(angle1) }));
    }
}

struct CapsuleRing {
    float x = 0.0f;
    float radius = 0.0f;
};

std::vector<CapsuleRing> capsuleRings(const RagdollCapsuleDefinition3D& capsule, int capSegments) {
    const float halfSegment = ragdollCapsuleHalfSegment3D(capsule);
    const float negativeRadius = capsule.radiusMeters;
    const float positiveRadius = ragdollCapsuleRadiusAtPositiveX3D(capsule);
    if (!(halfSegment > 0.000001f)) {
        return { { -negativeRadius, 0.0f }, { 0.0f, std::max(negativeRadius, positiveRadius) },
            { positiveRadius, 0.0f } };
    }

    const float centerDistance = 2.0f * halfSegment;
    const float radiusDifference = negativeRadius - positiveRadius;
    if (std::abs(radiusDifference) >= centerDistance) {
        const float center = negativeRadius >= positiveRadius ? -halfSegment : halfSegment;
        const float radius = std::max(negativeRadius, positiveRadius);
        return { { center - radius, 0.0f }, { center, radius }, { center + radius, 0.0f } };
    }
    const float tangentNormalX = radiusDifference / centerDistance;
    const float tangentAngle = std::acos(tangentNormalX);
    std::vector<CapsuleRing> rings;
    rings.reserve(static_cast<std::size_t>(capSegments * 2 + 3));
    for (int step = 0; step <= capSegments; ++step) {
        const float amount = static_cast<float>(step) / static_cast<float>(capSegments);
        const float angle = std::numbers::pi_v<float> + (tangentAngle - std::numbers::pi_v<float>)*amount;
        rings.push_back({ -halfSegment + negativeRadius * std::cos(angle), negativeRadius * std::sin(angle) });
    }
    // O primeiro anel da tampa positiva coincide geometricamente com o fim do
    // cone somente em capsulas uniformes; em capsulas conicas ele tem outro
    // centro e raio, e os dois aneis formam a lateral tangente.
    for (int step = capSegments; step >= 0; --step) {
        const float amount = static_cast<float>(step) / static_cast<float>(capSegments);
        const float angle = tangentAngle * amount;
        rings.push_back({ halfSegment + positiveRadius * std::cos(angle), positiveRadius * std::sin(angle) });
    }
    return rings;
}

void appendCapsule(
    std::vector<Segment>& segments, const RagdollLinkDefinition3D& link, int radialSegments, int capSegments) {
    const std::vector<CapsuleRing> rings = capsuleRings(link.collider, capSegments);
    for (const CapsuleRing& ring : rings) appendRing(segments, link, ring.x, ring.radius, radialSegments);

    constexpr std::array<float, 4> MeridianAngles = {
        0.0f,
        0.5f * std::numbers::pi_v<float>,
        std::numbers::pi_v<float>,
        1.5f * std::numbers::pi_v<float>,
    };
    for (std::size_t ring = 1; ring < rings.size(); ++ring) {
        for (const float angle : MeridianAngles) {
            const CapsuleRing& a = rings[ring - 1];
            const CapsuleRing& b = rings[ring];
            segments.emplace_back(toModel(link, { a.x, a.radius * std::cos(angle), a.radius * std::sin(angle) }),
                toModel(link, { b.x, b.radius * std::cos(angle), b.radius * std::sin(angle) }));
        }
    }
}

void appendBox(std::vector<Segment>& segments, const RagdollLinkDefinition3D& link) {
    const Vec3 half = link.collider.boxHalfExtents;
    std::array<Vec3, 8> corners;
    for (std::size_t index = 0; index < corners.size(); ++index) {
        corners[index] = toModel(link,
            { (index & 1u) != 0u ? half.x : -half.x, (index & 2u) != 0u ? half.y : -half.y,
                (index & 4u) != 0u ? half.z : -half.z });
    }
    constexpr std::array<std::array<std::size_t, 2>, 12> Edges = { {
        { 0, 1 },
        { 2, 3 },
        { 4, 5 },
        { 6, 7 },
        { 0, 2 },
        { 1, 3 },
        { 4, 6 },
        { 5, 7 },
        { 0, 4 },
        { 1, 5 },
        { 2, 6 },
        { 3, 7 },
    } };
    for (const auto& edge : Edges) segments.emplace_back(corners[edge[0]], corners[edge[1]]);
}

Vec3 posedPoint(const RagdollLinkDefinition3D& link, Vec3 restPoint, Vec3 posedPosition, Quaternion posedOrientation,
    Vec3 inPlaceShift) {
    const Vec3 linkLocal = link.modelOrientation.conjugate().rotate(restPoint - link.modelPosition);
    return posedPosition + posedOrientation.rotate(linkLocal) + inPlaceShift;
}

} // namespace

RagdollColliderWireframe3D buildRagdollColliderWireframe3D(
    const RagdollProfile3D& profile, int radialSegments, int capSegments) {
    validateRagdollProfileOrThrow3D(profile);
    if (radialSegments < 8 || radialSegments > 128)
        throw std::invalid_argument("radialSegments precisa estar entre 8 e 128");
    if (capSegments < 2 || capSegments > 32) throw std::invalid_argument("capSegments precisa estar entre 2 e 32");

    RagdollColliderWireframe3D result;
    result.profileId = profile.id;
    result.links.reserve(profile.links.size());
    for (const RagdollLinkDefinition3D& link : profile.links) {
        RagdollColliderWireframeLink3D wireframe;
        wireframe.linkId = link.id;
        wireframe.shape = link.collider.shape;
        wireframe.side = sideFor(link.id);
        wireframe.linkOriginModel = link.modelPosition;
        wireframe.colliderCenterModel = link.modelPosition + link.modelOrientation.rotate(link.collider.localPosition);
        wireframe.centerOfMassModel = link.modelPosition + link.modelOrientation.rotate(link.centerOfMassLocal);
        wireframe.hasInboundJoint = link.parentIndex >= 0;
        if (wireframe.hasInboundJoint) wireframe.inboundJointModel = link.inboundJoint.anchorModelPosition;
        if (link.collider.shape == RagdollColliderShape3D::Box)
            appendBox(wireframe.segments, link);
        else
            appendCapsule(wireframe.segments, link, radialSegments, capSegments);
        result.links.push_back(std::move(wireframe));
    }
    return result;
}

RagdollColliderWireframe3D poseRagdollColliderWireframe3D(const RagdollProfile3D& profile,
    const RagdollColliderWireframe3D& restWireframe, std::span<const Vec3> linkPositions,
    std::span<const Quaternion> linkOrientations, bool inPlace) {
    validateRagdollProfileOrThrow3D(profile);
    if (restWireframe.profileId != profile.id || restWireframe.links.size() != profile.links.size()) {
        throw std::invalid_argument("wireframe e perfil precisam representar os mesmos links");
    }
    if (linkPositions.size() != profile.links.size() || linkOrientations.size() != profile.links.size()) {
        throw std::invalid_argument("a pose precisa conter posicao e orientacao para todos os links");
    }

    Vec3 inPlaceShift;
    if (inPlace && !profile.links.empty()) {
        const Vec3 moved = linkPositions.front() - profile.links.front().modelPosition;
        inPlaceShift = { -moved.x, -moved.y, 0.0f };
    }

    RagdollColliderWireframe3D result = restWireframe;
    for (std::size_t index = 0; index < result.links.size(); ++index) {
        const RagdollLinkDefinition3D& profileLink = profile.links[index];
        RagdollColliderWireframeLink3D& posedLink = result.links[index];
        if (posedLink.linkId != profileLink.id) {
            throw std::invalid_argument("a ordem dos links do wireframe diverge do perfil");
        }
        const auto transform = [&](Vec3 point) {
            return posedPoint(profileLink, point, linkPositions[index], linkOrientations[index], inPlaceShift);
        };
        for (Segment& segment : posedLink.segments) {
            segment.first = transform(segment.first);
            segment.second = transform(segment.second);
        }
        posedLink.linkOriginModel = transform(posedLink.linkOriginModel);
        posedLink.colliderCenterModel = transform(posedLink.colliderCenterModel);
        posedLink.centerOfMassModel = transform(posedLink.centerOfMassModel);
        if (posedLink.hasInboundJoint) posedLink.inboundJointModel = transform(posedLink.inboundJointModel);
    }
    return result;
}

} // namespace MatterEngine
