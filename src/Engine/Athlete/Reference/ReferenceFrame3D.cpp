#include "Engine/Athlete/Reference/ReferenceFrame3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {
bool positiveFinite(float value) { return std::isfinite(value) && value > 0.0f; }
} // namespace

bool NavigationProfile3D::valid() const {
    return positiveFinite(acceleration) && positiveFinite(deceleration) && positiveFinite(brake)
        && positiveFinite(turnRate);
}

ReferenceFrame3D buildReferenceFrame3D(const ReferenceMotion3D& motion, const ReferenceFrameInput3D& input,
    BodyForecast3D& forecast) {
    ReferenceFrame3D frame;
    if (!motion.valid) return frame;
    frame.valid = true;
    frame.pose = motion.pose;
    frame.coordinates = motion.coordinates;
    frame.rootPosition = motion.rootPosition;
    frame.rootOrientation = motion.rootOrientation;
    frame.facingYaw = motion.facingYaw;
    frame.phase = motion.phase;
    frame.movingShare = motion.movingShare;
    frame.cycling = motion.cycling;
    frame.cycleSeconds = motion.cycleSeconds;
    frame.airborne = motion.airborne;
    frame.requestedVelocity = { input.requested.x, input.requested.y, 0.0f };
    frame.velocity = { input.velocity.x, input.velocity.y, 0.0f };

    // O plano a frente pelo perfil pedido (o mesmo previsor do manejo, com os
    // limites do perfil).
    BodyForecastInput3D plan;
    plan.pelvisGround = input.pelvisGround;
    plan.velocity = frame.velocity;
    plan.requested = frame.requestedVelocity;
    const NavigationProfile3D profile = input.profile.valid() ? input.profile : NavigationProfile3D {};
    plan.acceleration = profile.acceleration;
    plan.deceleration = profile.deceleration;
    plan.brake = profile.brake;
    plan.turnRate = profile.turnRate;
    plan.facingYaw = input.facingYaw;
    plan.facingRate = input.facingRate;
    plan.hipInBody = input.hipInBody;
    const float forecastSpan = static_cast<float>(ReferenceMotion3D::ForecastCount - 1) * motion.forecastStep;
    plan.horizon = std::max({ forecastSpan, frame.planTimes.back(), 0.1f });
    forecast.build(plan);
    for (int i = 0; i < ReferenceFrame3D::PlanCount; ++i) {
        const float t = frame.planTimes[static_cast<std::size_t>(i)];
        frame.planPosition[static_cast<std::size_t>(i)] = forecast.pelvisGround(t);
        frame.planVelocity[static_cast<std::size_t>(i)] = forecast.velocity(t);
    }

    // Os pes: o apoio pela fase da marcha (andando) ou pelo contato (parado) e
    // a meta de contato do balanco pela previsao do pe do estilo.
    for (std::size_t side = 0; side < 2; ++side) {
        ReferenceFootGoal3D& foot = frame.feet[side];
        foot.planted = motion.cycling ? motion.cyclePlanted[side] : motion.footPlanted[side];
        foot.secondsToLiftoff = motion.secondsToLiftoff[side];
        foot.secondsToTouchdown = motion.cycling ? motion.cycleSecondsToTouchdown[side] : motion.secondsToTouchdown[side];
        if (foot.planted || !motion.cycling || foot.secondsToTouchdown < 0.0f || motion.forecastStep <= 0.0f
            || motion.footForecast == nullptr)
            continue;
        if (foot.secondsToTouchdown > forecastSpan) continue;
        const int k = std::clamp(static_cast<int>(std::lround(foot.secondsToTouchdown / motion.forecastStep)), 0,
            ReferenceMotion3D::ForecastCount - 1);
        const Vec3 local = (*motion.footForecast)[side][static_cast<std::size_t>(k)];
        if (local.lengthSquared() <= 0.0f) continue;
        foot.touchdownWorld = forecast.toWorld(local, static_cast<float>(k) * motion.forecastStep);
        foot.touchdownValid = true;
    }
    return frame;
}

} // namespace MatterEngine
