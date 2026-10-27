#include "Engine/BodyControl/Planning/HopPlanner3D.hpp"

#include <cmath>

namespace MatterEngine {
namespace {
constexpr double Pi = 3.14159265358979323846;
}

double hopTakeoffSpeed3D(const HopSettings3D& settings) {
    return 0.5 * settings.gravity * settings.flightTime;
}

double hopPeakForce3D(const HopSettings3D& settings) {
    const double impulse = settings.mass * (2.0 * hopTakeoffSpeed3D(settings) + settings.gravity * settings.stanceTime);
    // Meio seno sobre o peso: a integral de sin(pi t/T) em [0,T] e 2T/pi.
    return settings.mass * settings.gravity + Pi * (impulse - settings.mass * settings.gravity * settings.stanceTime) / (2.0 * settings.stanceTime);
}

HopSample3D sampleHopStance3D(const HopSettings3D& settings, double time) {
    HopSample3D sample;
    if (time < 0.0 || time > settings.stanceTime || settings.stanceTime <= 0.0) {
        sample.acceleration = -settings.gravity;
        return sample;
    }
    const double weight = settings.mass * settings.gravity;
    sample.force = weight + (hopPeakForce3D(settings) - weight) * std::sin(Pi * time / settings.stanceTime);
    sample.acceleration = sample.force / settings.mass - settings.gravity;
    return sample;
}

} // namespace MatterEngine
