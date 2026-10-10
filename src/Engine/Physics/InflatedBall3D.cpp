#include "Engine/Physics/InflatedBall3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::InflatedBallModel3D {

namespace {
constexpr float Pi = 3.14159265358979f;
}

float restVolume(const InflatedBallProfile3D& profile) {
    const float r = profile.radiusMeters;
    return 4.0f / 3.0f * Pi * r * r * r;
}

float capVolume(const InflatedBallProfile3D& profile, float compression) {
    const float d = std::clamp(compression, 0.0f, profile.radiusMeters);
    return Pi * d * d * (3.0f * profile.radiusMeters - d) / 3.0f;
}

float patchArea(const InflatedBallProfile3D& profile, float compression) {
    const float d = std::clamp(compression, 0.0f, profile.radiusMeters);
    return Pi * std::max(0.0f, 2.0f * profile.radiusMeters * d - d * d);
}

float gaugePressure(const InflatedBallProfile3D& profile, float volume) {
    const float v0 = restVolume(profile);
    const float v = std::max(0.25f * v0, volume);
    const float absolute0 = profile.atmosphericPressurePascals + profile.gaugePressurePascals;
    return absolute0 * std::pow(v0 / v, profile.adiabaticExponent)
        - profile.atmosphericPressurePascals;
}

float normalForce(const InflatedBallProfile3D& profile, float compression,
    float approachSpeed, float volume, float extraDampingRatio) {
    if (compression <= 0.0f) return 0.0f;
    const float r = profile.radiusMeters;
    const float d = compression;
    const float gauge = gaugePressure(profile, volume);
    const float patch = patchArea(profile, d);
    const float elastic = gauge * patch + profile.carcassStiffness * d * std::sqrt(d);
    // Rigidez instantanea dF/dd: o ar sobre a mancha que cresce, o ar que
    // sobe de pressao com o volume perdido, e a casca.
    const float v0 = restVolume(profile);
    const float v = std::max(0.25f * v0, volume);
    const float absolute = gauge + profile.atmosphericPressurePascals;
    const float stiffness = gauge * Pi * std::max(0.0f, 2.0f * r - 2.0f * d)
        + profile.adiabaticExponent * absolute / v * patch * patch
        + 1.5f * profile.carcassStiffness * std::sqrt(d);
    const float slow = std::clamp(std::abs(approachSpeed) / std::max(0.01f, profile.lowSpeedDampingSpeed), 0.0f, 1.0f);
    const float zeta = profile.dampingRatio + profile.lowSpeedDampingRatio * (1.0f - slow * slow * (3.0f - 2.0f * slow))
        + extraDampingRatio;
    const float pressureShare = profile.dampingReferencePressurePascals > 0.0f
        ? profile.dampingReferencePressurePascals / std::max(1000.0f, profile.gaugePressurePascals) : 1.0f;
    const float damping = 2.0f * zeta * std::sqrt(profile.massKg * std::max(0.0f, stiffness) * pressureShare);
    return std::max(0.0f, elastic + damping * approachSpeed);
}

Vec3 aerodynamicForce(const InflatedBallProfile3D& profile, Vec3 relativeVelocity,
    Vec3 angularVelocity) {
    const float speed = relativeVelocity.length();
    if (speed < 0.05f) return {};
    const float r = profile.radiusMeters;
    const float area = Pi * r * r;
    const float reynolds = speed * 2.0f * r / std::max(1e-7f, profile.airKinematicViscosity);
    const float crisis = std::clamp((reynolds - profile.criticalReynolds)
        / std::max(1.0f, profile.criticalReynoldsSpan), 0.0f, 1.0f);
    const float spin = angularVelocity.length();
    const float spinParameter = spin * r / speed;
    const float drag = profile.dragSubcritical
        + (profile.dragSupercritical - profile.dragSubcritical) * crisis
        + profile.dragSpinGain * std::min(spinParameter, 1.0f) * crisis;
    const float dynamicPressure = 0.5f * profile.airDensity * area * speed * speed;
    const Vec3 along = relativeVelocity * (1.0f / speed);
    Vec3 force = along * (-drag * dynamicPressure);
    if (spin > 1e-3f) {
        const Vec3 lift = cross(angularVelocity, relativeVelocity);
        const float liftLength = lift.length();
        if (liftLength > 1e-6f) {
            const float coefficient = std::min(profile.liftMaximum, profile.liftSlope * spinParameter);
            force += lift * (coefficient * dynamicPressure / liftLength);
        }
    }
    return force;
}

float shellInertia(const InflatedBallProfile3D& profile) {
    return 2.0f / 3.0f * profile.massKg * profile.radiusMeters * profile.radiusMeters;
}

InflatedBallProfile3D scaledProfile(const InflatedBallProfile3D& base, float radiusMeters) {
    InflatedBallProfile3D profile = base;
    const float k = std::max(0.1f, radiusMeters) / std::max(0.01f, base.radiusMeters);
    profile.radiusMeters = base.radiusMeters * k;
    profile.massKg = base.massKg * k * k;
    profile.gaugePressurePascals = base.gaugePressurePascals / k;
    profile.dampingReferencePressurePascals = base.dampingReferencePressurePascals / k;
    profile.rollingResistanceMeters = base.rollingResistanceMeters * k;
    return profile;
}

} // namespace MatterEngine::InflatedBallModel3D
