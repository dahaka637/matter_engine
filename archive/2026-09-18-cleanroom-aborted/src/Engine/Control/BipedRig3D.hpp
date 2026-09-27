#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace MatterEngine {

struct BipedRig3D {
    bool valid = false;
    std::size_t pelvis = 0;
    std::size_t torso = InvalidBipedLink3D;
    std::size_t head = InvalidBipedLink3D;
    std::array<std::size_t, 2> foot {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::array<std::size_t, 2> lowerLeg {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::array<std::size_t, 2> upperLeg {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::array<std::size_t, 2> hand {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::array<std::size_t, 2> lowerArm {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::array<std::size_t, 2> upperArm {
        InvalidBipedLink3D, InvalidBipedLink3D
    };
    std::vector<std::size_t> spine;
    float legLengthMeters = 1.0f;
    float nominalStepWidthMeters = 0.22f;
    float shoulderWidthMeters = 0.40f;
};

[[nodiscard]] BipedRig3D resolveBipedRig3D(
    const RagdollProfile3D& profile);
[[nodiscard]] bool bipedLinkIsDescendant3D(
    const RagdollProfile3D& profile, std::size_t link,
    std::size_t ancestor);
[[nodiscard]] Vec3 bipedAnatomicalAxisWorld3D(
    const RagdollLinkDefinition3D& definition,
    const PhysicsBodyState3D& state, Vec3 modelAxis);
[[nodiscard]] Vec3 bipedColliderSupportOffsetLocal3D(
    const RagdollLinkDefinition3D& link,
    Quaternion linkWorldOrientation, Vec3 worldSupportNormal);
[[nodiscard]] Vec3 bipedSoleWorld3D(
    const RagdollLinkDefinition3D& link,
    const PhysicsBodyState3D& state, Vec3 worldSupportNormal);
[[nodiscard]] Quaternion bipedNeutralLinkOrientationWorld3D(
    const RagdollLinkDefinition3D& link, float headingRadians,
    Vec3 groundNormal = {0.0f, 0.0f, 1.0f});

} // namespace MatterEngine
