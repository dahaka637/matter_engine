#include "Engine/Control/BipedRig3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine {
namespace {

std::string lower(std::string_view value) {
    std::string result(value);
    std::transform(result.begin(), result.end(), result.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return result;
}

bool hasAny(std::string_view id,
    std::initializer_list<std::string_view> words) {
    const std::string text = lower(id);
    for (const auto word : words) {
        if (text.find(std::string(word)) != std::string::npos) return true;
    }
    return false;
}

int explicitSide(std::string_view id) {
    const std::string text = lower(id);
    if (text.find("left") != std::string::npos
        || text.find("_l") != std::string::npos
        || text.find(".l") != std::string::npos
        || text.rfind("l_", 0) == 0
        || text.rfind("l.", 0) == 0) {
        return 0;
    }
    if (text.find("right") != std::string::npos
        || text.find("_r") != std::string::npos
        || text.find(".r") != std::string::npos
        || text.rfind("r_", 0) == 0
        || text.rfind("r.", 0) == 0) {
        return 1;
    }
    return -1;
}

std::array<std::size_t, 2> pairCandidates(
    const RagdollProfile3D& profile,
    std::initializer_list<std::string_view> words) {
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (hasAny(profile.links[i].id, words)) candidates.push_back(i);
    }
    std::array<std::size_t, 2> result{
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    for (std::size_t index : candidates) {
        const int side = explicitSide(profile.links[index].id);
        if (side >= 0 && result[static_cast<std::size_t>(side)]
                == InvalidBipedLink3D) {
            result[static_cast<std::size_t>(side)] = index;
        }
    }
    if (result[0] == InvalidBipedLink3D
        || result[1] == InvalidBipedLink3D) {
        if (candidates.size() >= 2) {
            std::sort(candidates.begin(), candidates.end(),
                [&](std::size_t a, std::size_t b) {
                    return profile.links[a].modelPosition.y
                        > profile.links[b].modelPosition.y;
                });
            if (result[0] == InvalidBipedLink3D) result[0] = candidates.front();
            if (result[1] == InvalidBipedLink3D) result[1] = candidates.back();
        }
    }
    return result;
}

std::size_t highestCandidate(const RagdollProfile3D& profile,
    std::initializer_list<std::string_view> words) {
    std::size_t result = InvalidBipedLink3D;
    float best = -1.0e30f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (!hasAny(profile.links[i].id, words)) continue;
        if (profile.links[i].modelPosition.z > best) {
            best = profile.links[i].modelPosition.z;
            result = i;
        }
    }
    return result;
}

float distance(Vec3 a, Vec3 b) {
    return (a-b).length();
}

Vec3 neutralSoleModel(const RagdollLinkDefinition3D& link) {
    const Quaternion q = link.modelOrientation;
    const Vec3 offset = bipedColliderSupportOffsetLocal3D(
        link, q, {0.0f, 0.0f, 1.0f});
    return link.modelPosition + q.rotate(offset);
}

}

BipedRig3D resolveBipedRig3D(const RagdollProfile3D& profile) {
    BipedRig3D rig;
    if (profile.links.empty()) return rig;
    rig.pelvis = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].parentIndex < 0) {
            rig.pelvis = i;
            break;
        }
    }

    rig.foot = pairCandidates(profile, {"foot", "ankle"});
    rig.hand = pairCandidates(profile, {"hand", "wrist"});
    rig.torso = highestCandidate(profile, {"chest", "torso"});
    if (rig.torso == InvalidBipedLink3D)
        rig.torso = highestCandidate(profile, {"spine", "lumbar"});
    if (rig.torso == InvalidBipedLink3D) rig.torso = rig.pelvis;
    rig.head = highestCandidate(profile, {"head"});

    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t foot = rig.foot[side];
        if (foot != InvalidBipedLink3D
            && profile.links[foot].parentIndex >= 0) {
            rig.lowerLeg[side] =
                static_cast<std::size_t>(profile.links[foot].parentIndex);
            const int upper =
                profile.links[rig.lowerLeg[side]].parentIndex;
            if (upper >= 0)
                rig.upperLeg[side] = static_cast<std::size_t>(upper);
        }
        const std::size_t hand = rig.hand[side];
        if (hand != InvalidBipedLink3D
            && profile.links[hand].parentIndex >= 0) {
            rig.lowerArm[side] =
                static_cast<std::size_t>(profile.links[hand].parentIndex);
            const int upper =
                profile.links[rig.lowerArm[side]].parentIndex;
            if (upper >= 0)
                rig.upperArm[side] = static_cast<std::size_t>(upper);
        }
    }

    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        if (hasAny(profile.links[i].id,
                {"spine", "lumbar", "chest", "torso"})) {
            rig.spine.push_back(i);
        }
    }

    if (rig.foot[0] != InvalidBipedLink3D
        && rig.foot[1] != InvalidBipedLink3D
        && rig.upperLeg[0] != InvalidBipedLink3D
        && rig.upperLeg[1] != InvalidBipedLink3D) {
        const auto& upper = profile.links[rig.upperLeg[0]];
        const auto& lower = profile.links[rig.lowerLeg[0]];
        const auto& foot = profile.links[rig.foot[0]];
        const Vec3 hip = upper.inboundJoint.anchorModelPosition;
        const Vec3 knee = lower.inboundJoint.anchorModelPosition;
        const Vec3 ankle = foot.inboundJoint.anchorModelPosition;
        const Vec3 sole = neutralSoleModel(foot);
        const float length =
            distance(hip, knee)+distance(knee, ankle)+distance(ankle, sole);
        if (std::isfinite(length) && length > 0.25f)
            rig.legLengthMeters = length;

        const Vec3 soleL = neutralSoleModel(profile.links[rig.foot[0]]);
        const Vec3 soleR = neutralSoleModel(profile.links[rig.foot[1]]);
        const float footWidth = std::abs(soleL.y-soleR.y);
        const float hipWidth = std::abs(
            profile.links[rig.upperLeg[0]].inboundJoint.anchorModelPosition.y
            - profile.links[rig.upperLeg[1]]
                .inboundJoint.anchorModelPosition.y);
        rig.nominalStepWidthMeters =
            std::max({0.12f, footWidth, hipWidth*0.90f});
    }

    if (rig.hand[0] != InvalidBipedLink3D
        && rig.hand[1] != InvalidBipedLink3D) {
        rig.shoulderWidthMeters = std::max(0.20f,
            std::abs(profile.links[rig.hand[0]].modelPosition.y
                - profile.links[rig.hand[1]].modelPosition.y)*0.45f);
    }

    rig.valid = rig.foot[0] != InvalidBipedLink3D
        && rig.foot[1] != InvalidBipedLink3D
        && rig.upperLeg[0] != InvalidBipedLink3D
        && rig.upperLeg[1] != InvalidBipedLink3D
        && rig.torso != InvalidBipedLink3D;
    return rig;
}

bool bipedLinkIsDescendant3D(const RagdollProfile3D& profile,
    std::size_t link, std::size_t ancestor) {
    if (link >= profile.links.size() || ancestor >= profile.links.size())
        return false;
    int current = static_cast<int>(link);
    while (current >= 0) {
        if (static_cast<std::size_t>(current) == ancestor) return true;
        current = profile.links[static_cast<std::size_t>(current)].parentIndex;
    }
    return false;
}

Vec3 bipedAnatomicalAxisWorld3D(
    const RagdollLinkDefinition3D& definition,
    const PhysicsBodyState3D& state, Vec3 modelAxis) {
    const Vec3 local =
        definition.modelOrientation.conjugate().rotate(modelAxis).normalized();
    return state.orientation.rotate(local).normalized();
}

Vec3 bipedColliderSupportOffsetLocal3D(
    const RagdollLinkDefinition3D& link,
    Quaternion linkWorldOrientation, Vec3 worldSupportNormal) {
    const auto& collider = link.collider;
    if (worldSupportNormal.lengthSquared() < 1.0e-10f)
        worldSupportNormal = {0.0f, 0.0f, 1.0f};
    const Vec3 nLink =
        linkWorldOrientation.conjugate().rotate(
            worldSupportNormal.normalized());
    const Vec3 nCollider =
        collider.localOrientation.conjugate().rotate(nLink).normalized();

    Vec3 supportCollider;
    if (collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 h = collider.boxHalfExtents;
        supportCollider = {
            nCollider.x >= 0.0f ? -h.x : h.x,
            nCollider.y >= 0.0f ? -h.y : h.y,
            nCollider.z >= 0.0f ? -h.z : h.z
        };
    } else {
        const float radius = std::max(0.0f, collider.radiusMeters);
        const float halfCylinder = std::max(
            0.0f, collider.lengthMeters*0.5f-radius);
        supportCollider =
            Vec3{nCollider.x >= 0.0f ? -halfCylinder : halfCylinder,
                 0.0f, 0.0f}
            - nCollider*radius;
    }
    return collider.localPosition
        + collider.localOrientation.rotate(supportCollider);
}

Vec3 bipedSoleWorld3D(const RagdollLinkDefinition3D& link,
    const PhysicsBodyState3D& state, Vec3 worldSupportNormal) {
    return state.position + state.orientation.rotate(
        bipedColliderSupportOffsetLocal3D(
            link, state.orientation, worldSupportNormal));
}

Quaternion bipedNeutralLinkOrientationWorld3D(
    const RagdollLinkDefinition3D& link, float headingRadians,
    Vec3 groundNormal) {
    Quaternion result =
        (yawQuaternionBiped(headingRadians)*link.modelOrientation).normalized();
    const Vec3 localModelUp =
        link.modelOrientation.conjugate().rotate({0.0f, 0.0f, 1.0f});
    return alignOrientationUpBiped(
        result, localModelUp, groundNormal).normalized();
}

} // namespace MatterEngine
