#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

struct BipedJointTorqueField3D {
    std::vector<std::array<float, 3>> torqueNewtonMeters;

    void reset(std::size_t linkCount);
    void clear();
};

class BipedDynamicsAdapter3D {
public:
    [[nodiscard]] bool validFor(
        const RagdollProfile3D& profile,
        const RagdollDynamics3D& dynamics) const;

    void addComForceThroughSupport(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        std::size_t supportLink,
        Vec3 supportPointWorld,
        Vec3 forceWorld,
        BipedJointTorqueField3D& field,
        std::span<const std::uint8_t> enabledLinks = {}) const;

    void addComForceThroughContacts(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        std::span<const BipedSupportContact3D> contacts,
        Vec3 forceWorld,
        BipedJointTorqueField3D& field,
        std::span<const std::uint8_t> enabledLinks = {}) const;

    void addAngularTaskThroughSupport(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        std::size_t taskLink,
        std::size_t supportLink,
        Vec3 torqueWorld,
        BipedJointTorqueField3D& field,
        std::span<const std::uint8_t> enabledLinks = {}) const;

    void addAngularTaskThroughContacts(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        std::size_t taskLink,
        std::span<const BipedSupportContact3D> contacts,
        Vec3 torqueWorld,
        BipedJointTorqueField3D& field,
        std::span<const std::uint8_t> enabledLinks = {}) const;

private:
    void mapGeneralizedTorqueToJoints(
        const RagdollProfile3D& profile,
        const RagdollDynamics3D& dynamics,
        std::span<const float> generalizedTorque,
        BipedJointTorqueField3D& field,
        std::span<const std::uint8_t> enabledLinks) const;
};

} // namespace MatterEngine
