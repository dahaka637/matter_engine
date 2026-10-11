#include "Engine/Athlete/Body/PhysicalFeetAssist3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 clampLength(Vec3 value, float maximum) {
    const float length = value.length();
    return length > maximum && length > 1e-9f ? value * (maximum / length) : value;
}

// O vetor de rotacao (eixo vezes angulo) de um quaternion, pelo caminho curto.
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

// O que falta do pedido depois do que ja foi dado, so no rumo do pedido (o
// que a pelve deu de lado ou a mais nao e desfeito pelos pes).
Vec3 missing(Vec3 wanted, Vec3 given) {
    const float length = wanted.length();
    if (length < 1e-6f) return {};
    const Vec3 direction = wanted * (1.0f / length);
    return direction * std::max(0.0f, length - dot(given, direction));
}

} // namespace

PhysicalFeetAssistOutput3D physicalFeetAssist3D(const RagdollProfile3D& profile, const RagdollState3D& state,
    const BodyObservation3D& observation, const PhysicalFeetAssistInput3D& input,
    const PhysicalFeetAssistSettings3D& settings) {
    PhysicalFeetAssistOutput3D output;
    const float share = std::clamp(input.share, 0.0f, 1.0f);
    if (share <= 0.0f || !observation.valid || observation.supportCount == 0
        || state.links.size() != profile.links.size() || profile.links.empty())
        return output;
    output.supportCount = observation.supportCount;
    const float mass = observation.massKilograms;
    // O servo cheio: a pelve na referencia (posicao e velocidade) e a
    // orientacao dela.
    const Vec3 positionError = input.referencePosition - observation.pelvisPosition;
    const Vec3 velocityError = input.referenceVelocity - observation.pelvisVelocity;
    const float omega = settings.rootFrequency;
    const Vec3 acceleration = clampLength(positionError * (omega * omega)
            + velocityError * (2.0f * settings.rootDamping * omega),
        settings.maximumRootAcceleration);
    const Vec3 force = acceleration * mass;
    const Vec3 orientationError = rotationVector(input.referenceOrientation * observation.pelvisOrientation.conjugate());
    const float turn = settings.orientationFrequency;
    const Vec3 angular = orientationError * (turn * turn)
        + (input.referenceAngularVelocity - observation.pelvisAngularVelocity) * (2.0f * settings.orientationDamping * turn);
    const Vec3 torque { angular.x * settings.bodyInertiaHorizontal, angular.y * settings.bodyInertiaHorizontal,
        angular.z * settings.bodyInertiaVertical };
    output.requestedForce = force;
    // So o que a pelve nao deu, na fracao escolhida.
    SupportWrenchRequest3D request;
    request.bodyLink = 0;
    request.force = missing(force, input.pelvisForce) * share;
    // (Os pes so empurram: puxar a pelve para baixo tiraria o pe do chao.)
    request.force.z = std::max(0.0f, request.force.z);
    request.torque = missing(torque, input.pelvisTorque) * share;
    request.weightNewtons = mass * 9.81f;
    if (request.force.lengthSquared() < 1e-6f && request.torque.lengthSquared() < 1e-6f) return output;
    std::vector<Vec3> centers(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        centers[i] = state.links[i].position + state.links[i].orientation.rotate(profile.links[i].centerOfMassLocal);
    request.bodyPoint = centers.front();
    SupportWrenchResult3D routed = routeSupportWrench3D(request, observation, centers, settings.support);
    output.wrenches = std::move(routed.wrenches);
    output.routedForce = request.force;
    output.deliveredForce = routed.deliveredForce;
    return output;
}

} // namespace MatterEngine
