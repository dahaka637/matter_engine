#include "Engine/BodyControl/Planning/HopPlanner3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
constexpr double Pi = 3.14159265358979323846;
}

double hopTakeoffSpeed3D(const HopSettings3D& settings) {
    return 0.5 * settings.gravity * settings.flightTime;
}

double hopPeakForce3D(const HopSettings3D& settings, double entrySpeed) {
    // O impulso m (v0 - v_entrada + g T); a integral de sin(pi t/T) em [0,T] e 2T/pi.
    const double impulse = settings.mass * (hopTakeoffSpeed3D(settings) - entrySpeed + settings.gravity * settings.stanceTime);
    return std::max(0.0, Pi * impulse / (2.0 * settings.stanceTime));
}

HopSample3D sampleHopStance3D(const HopSettings3D& settings, double time, double entrySpeed) {
    HopSample3D sample;
    const double g = settings.gravity, T = settings.stanceTime;
    const double t = std::clamp(time, 0.0, T);
    const double peak = T > 0.0 ? hopPeakForce3D(settings, entrySpeed) / settings.mass : 0.0;
    const double phase = T > 0.0 ? Pi * t / T : 0.0;
    sample.force = time >= 0.0 && time <= T ? settings.mass * peak * std::sin(phase) : 0.0;
    sample.acceleration = sample.force / settings.mass - g;
    sample.velocity = entrySpeed + peak * (T / Pi) * (1.0 - std::cos(phase)) - g * t;
    sample.displacement = entrySpeed * t + peak * (T / Pi) * (t - (T / Pi) * std::sin(phase)) - 0.5 * g * t * t;
    return sample;
}

} // namespace MatterEngine
