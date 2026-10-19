#include "Engine/Tracking/TrackingLeader3D.hpp"

#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

} // namespace

TrackingLeader3D::TrackingLeader3D(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& defaultClips,
    const AthleteRuntimeSettings3D& settings)
    : m_profile(&profile), m_clips(&defaultClips) {
    // A resposta da capsula do padrao (a regua "Resposta" do jogo).
    const AthleteCapsuleResponse3D response =
        athleteCapsuleResponse3D(settings.capsuleResponsePercent, settings.capsuleReversalPercent);
    m_response.acceleration = response.acceleration;
    m_response.deceleration = response.deceleration;
    m_response.brake = response.brake;
    m_response.turnRate = response.turnRate;
    athleteConfigureDefaultLocomotion3D(m_locomotion.settings(), settings);
    reset({}, 0.0f);
}

void TrackingLeader3D::reset(Vec3 groundPosition, float facingYawRadians) {
    m_locomotion.reset(facingYawRadians);
    m_capsulePosition = groundPosition;
    m_capsuleVelocity = {};
    m_hasPrevious = false;
    m_state = TrackingLeaderState3D {};
}

void TrackingLeader3D::step(const TrackingCommand3D& command, float deltaTime) {
    const RagdollProfile3D& profile = *m_profile;
    const float dt = std::max(1e-4f, deltaTime);
    // A capsula (pela velocidade media do passo, como a previsao).
    const Vec3 desired = planar(command.requestedVelocityWorld);
    const Vec3 next = bodyForecastCapsuleStep3D(m_capsuleVelocity, desired, m_response, dt);
    m_capsulePosition += (m_capsuleVelocity + next) * (0.5f * dt);
    m_capsuleVelocity = next;
    DefaultLocomotionInput3D input;
    input.feetPositionWorld = m_capsulePosition;
    input.velocityWorld = m_capsuleVelocity;
    input.grounded = true;
    input.requestedVelocityWorld = desired;
    input.facingYawRadians = command.facingYawRadians;
    input.deltaTime = dt;
    m_locomotion.update(profile, *m_clips, input);
    const DefaultLocomotionOutput3D& out = m_locomotion.output();
    m_state.valid = out.valid;
    m_state.requestedVelocityWorld = desired;
    if (!out.valid) return;

    m_state.rootPosition = out.rootPosition;
    m_state.rootOrientation = out.rootOrientation.normalized();
    m_state.coordinates = out.coordinates;
    m_state.coordinates.resize(profile.links.size());
    m_state.coordinateVelocities.assign(profile.links.size(), Vec3 {});
    Vec3 rootLinear, rootAngular;
    if (m_hasPrevious) {
        // As velocidades por diferenca finita entre ticks (o padrao nao as da).
        const float inverse = 1.0f / dt;
        for (std::size_t i = 0; i < m_state.coordinates.size() && i < m_previousCoordinates.size(); ++i)
            m_state.coordinateVelocities[i] = (m_state.coordinates[i] - m_previousCoordinates[i]) * inverse;
        rootLinear = (m_state.rootPosition - m_previousRoot) * inverse;
        rootAngular = rlQuaternionLog3D((m_state.rootOrientation * m_previousRootOrientation.conjugate()).normalized())
            * inverse;
    }
    rlBodyLinkStates3D(profile, m_state.rootPosition, m_state.rootOrientation, rootLinear, rootAngular,
        m_state.coordinates, m_state.coordinateVelocities, m_state.links);
    m_state.centerOfMass = rlBodyCenterOfMass3D(profile, m_state.links);
    m_state.centerOfMassVelocity = rlBodyCenterOfMassVelocity3D(profile, m_state.links);
    m_state.headingYawRadians = out.facingYawRadians;
    m_previousRoot = m_state.rootPosition;
    m_previousRootOrientation = m_state.rootOrientation;
    m_previousCoordinates = m_state.coordinates;
    m_hasPrevious = true;
}

} // namespace MatterEngine
