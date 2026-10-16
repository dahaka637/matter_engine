#include "Engine/Athlete/Legacy/LegacyReference3D.hpp"

namespace MatterEngine {

static_assert(ReferenceMotion3D::ForecastCount == DefaultLocomotionOutput3D::ForecastCount,
    "a previsao do pe da referencia e a do padrao tem o mesmo tamanho");

ReferenceMotion3D legacyReferenceMotion3D(const DefaultLocomotionOutput3D& output) {
    ReferenceMotion3D motion;
    motion.valid = output.valid;
    motion.pose = &output.pose;
    motion.coordinates = &output.coordinates;
    motion.rootPosition = output.rootPosition;
    motion.rootOrientation = output.rootOrientation;
    motion.facingYaw = output.facingYawRadians;
    motion.phase = output.cyclePhase;
    motion.movingShare = output.movingShare;
    motion.cycleSeconds = output.cycleSeconds;
    motion.cycling = output.cycling;
    motion.airborne = output.airborne;
    motion.cyclePlanted = output.cyclePlanted;
    motion.footPlanted = output.footPlanted;
    motion.secondsToLiftoff = output.secondsToLiftoff;
    motion.cycleSecondsToTouchdown = output.cycleSecondsToTouchdown;
    motion.secondsToTouchdown = output.secondsToTouchdown;
    motion.forecastStep = output.forecastStep;
    motion.footForecast = &output.footForecast;
    return motion;
}

} // namespace MatterEngine
