// A tensao nos limites das juntas (RagdollLimitTension3D, 27/10): o jogador mole
// leva um empurrao no peito e cai; com a tensao, as juntas batem menos no limite
// duro, o tranco do batente cai e o corpo segue estavel. Com MATTER_DUMP_DIR, grava
// as duas quedas (sem.dump e com.dump) no formato das folhas de quadros.
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Fall {
    int limitTicks = 0;           // (tick, junta, eixo) a menos de 0,02 rad do limite duro
    float peakLimitTorque = 0.0f; // o maior torque do batente (N.m)
    float peakSpin = 0.0f;        // a maior velocidade angular de um osso (rad/s)
    bool finite = true;
};

Fall fall(RagdollProfile3D profile, bool tension, const std::string& dump) {
    profile.limitTension.enabled = tension;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    PhysicsBodyDefinition3D floor;
    floor.motionType = PhysicsMotionType3D::Static;
    floor.position = { 0, 0, -0.1f };
    floor.materialId = "concrete";
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 20, 20, 0.1f };
    shape.materialId = "concrete";
    require(static_cast<bool>(scene->createBody(floor, std::span<const PhysicsShape3D>(&shape, 1))), "sem chao");
    RagdollSpawnDefinition3D spawn;
    spawn.pelvisPosition = { 0, 0, profile.standingRootHeightMeters };
    spawn.active = false;
    spawn.rigidityPercent = 15.0f;
    const auto handle = scene->createRagdoll(profile, spawn);
    std::uint32_t chest = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == "UpperChest") chest = static_cast<std::uint32_t>(i);
    std::ofstream out;
    if (!dump.empty()) {
        out.open(dump);
        out << "S " << (tension ? "com tensao" : "sem tensao") << '\n';
    }
    constexpr float dt = 1.0f / 120.0f;
    Fall result;
    for (int tick = 0; tick < 360; ++tick) {
        const float t = static_cast<float>(tick) * dt;
        // O empurrao: 0,15 s para tras e um pouco de lado, no peito alto.
        if (t >= 0.10f && t < 0.25f) scene->applyRagdollLinkForce(handle, chest, { -520.0f, 160.0f, 0.0f }, {});
        scene->simulate(dt);
        const RagdollState3D state = scene->ragdollState(handle);
        for (std::size_t i = 1; i < profile.links.size() && i < state.joints.size(); ++i)
            for (int a = 0; a < 3; ++a) {
                const auto& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
                if (!axis.enabled) continue;
                const float angle = state.joints[i].positionRadians[static_cast<std::size_t>(a)];
                if (angle > axis.maximumRadians - 0.02f || angle < axis.minimumRadians + 0.02f) ++result.limitTicks;
                result.peakLimitTorque = std::max(result.peakLimitTorque,
                    std::abs(state.joints[i].constraintTorqueNewtonMeters[static_cast<std::size_t>(a)]));
            }
        for (const PhysicsBodyState3D& link : state.links) {
            const float spin = link.angularVelocity.length();
            if (!std::isfinite(spin) || !std::isfinite(link.position.z)) result.finite = false;
            result.peakSpin = std::max(result.peakSpin, spin);
        }
        if (out && tick % 4 == 3) {
            out << "F " << (t + dt) << " 0\n";
            for (std::size_t i = 0; i < profile.links.size() && i < state.links.size(); ++i) {
                const RagdollLinkDefinition3D& link = profile.links[i];
                const Vec3 center = state.links[i].position + state.links[i].orientation.rotate(link.collider.localPosition);
                const Quaternion q = (state.links[i].orientation * link.collider.localOrientation).normalized();
                out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
                    << center.y << ' ' << center.z << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w << '\n';
            }
        }
    }
    return result;
}

} // namespace

int main() {
    try {
        const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
        const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
        const char* dir = std::getenv("MATTER_DUMP_DIR");
        const std::string base = dir ? std::string(dir) + "/" : std::string();
        const Fall off = fall(character.profile, false, dir ? base + "sem.dump" : std::string());
        const Fall on = fall(character.profile, true, dir ? base + "com.dump" : std::string());
        std::cout << "sem tensao: no limite " << off.limitTicks << ", pico do batente " << off.peakLimitTorque
                  << " N.m, giro maximo " << off.peakSpin << " rad/s\n"
                  << "com tensao: no limite " << on.limitTicks << ", pico do batente " << on.peakLimitTorque
                  << " N.m, giro maximo " << on.peakSpin << " rad/s\n";
        require(off.finite && on.finite, "a queda gerou numero invalido");
        require(on.peakSpin < 40.0f, "com a tensao um osso girou rapido demais (instavel)");
        require(on.limitTicks < off.limitTicks, "com a tensao as juntas deveriam ficar menos no limite duro");
        require(on.peakLimitTorque < off.peakLimitTorque, "com a tensao o tranco do batente deveria cair");
    } catch (const std::exception& error) {
        std::cerr << "Ragdoll limit tension tests failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Ragdoll limit tension tests passed\n";
    return 0;
}
