#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include <memory>

namespace MatterEngine {

// Contact constrained, torque actuated biped. No animation or root wrench.
class BipedController3D {
public:
    enum class Phase { Standing, Transfer, Swing, Landing, Falling, Recovering };
    struct Telemetry {
        Phase phase = Phase::Standing;
        Vec3 com, velocity;
        float height = 0, upright = 0;
        float footLoad[2] {};
        float dynamicsResidual = 0, constraintViolation = 0;
        int steps = 0, solverIterations = 0;
        bool solved = false;
    };
    BipedController3D();
    ~BipedController3D();
    BipedController3D(BipedController3D&&) noexcept;
    BipedController3D& operator=(BipedController3D&&) noexcept;
    void reset(const RagdollProfile3D&, const RagdollState3D&);
    void setVelocity(Vec3 velocityBody);
    void update(const RagdollProfile3D&, const RagdollState3D&,
        const RagdollDynamics3D&, float dt, bool manipulated = false);
    const std::vector<RagdollDriveTarget3D>& targets() const;
    const Telemetry& telemetry() const;
private:
    struct Impl;
    std::unique_ptr<Impl> m;
};
}
