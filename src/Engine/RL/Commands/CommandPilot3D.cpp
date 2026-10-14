#include "Engine/RL/Commands/CommandPilot3D.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace MatterEngine {
namespace {

constexpr float Pi = std::numbers::pi_v<float>;

float wrap(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

float pick(std::mt19937_64& rng, PilotRange3D range) {
    if (!(range.maximum > range.minimum)) return range.minimum;
    return std::uniform_real_distribution<float>(range.minimum, range.maximum)(rng);
}

bool chance(std::mt19937_64& rng, float fraction) {
    return std::uniform_real_distribution<float>(0.0f, 1.0f)(rng) < fraction;
}

const PilotBand3D* pickBand(const PilotScript3D& script, std::mt19937_64& rng) {
    if (script.bands.empty()) return nullptr;
    float total = 0.0f;
    for (const PilotBand3D& band : script.bands) total += std::max(0.0f, band.weight);
    float roll = std::uniform_real_distribution<float>(0.0f, std::max(1e-6f, total))(rng);
    for (const PilotBand3D& band : script.bands)
        if ((roll -= std::max(0.0f, band.weight)) < 0.0f) return &band;
    return &script.bands.back();
}

void markChange(PilotState3D& state) {
    state.changed = true;
    state.lastChange = state.clock;
    ++state.changes;
}

void startTurn(const PilotScript3D& script, PilotState3D& state, std::mt19937_64& rng) {
    const float sign = chance(rng, 0.5f) ? 1.0f : -1.0f;
    state.cameraTurnLeft = sign * pick(rng, script.camera.turnRadians);
    state.cameraTurnRate = pick(rng, script.camera.turnRate);
    markChange(state);
}

// Uma ponta da estrela (fromTip: de ponta a ponta, nunca a mesma).
void pickTip(const PilotScript3D& script, PilotState3D& state, std::mt19937_64& rng, bool fromTip) {
    const std::uint32_t directions = std::max(1u, script.starDirections);
    std::uint32_t slot = std::uniform_int_distribution<std::uint32_t>(0, directions - 1)(rng);
    if (fromTip && slot == state.lastSlot) slot = (slot + 1 + directions / 2) % directions;
    state.lastSlot = slot;
    const float jitter = std::uniform_real_distribution<float>(-1.0f, 1.0f)(rng) * script.starJitterDegrees * Pi / 180.0f;
    const float angle = state.startHeading + 2.0f * Pi * static_cast<float>(slot) / static_cast<float>(directions) + jitter;
    const PilotBand3D* band = pickBand(script, rng);
    const float distance = band ? pick(rng, band->distance) : 8.0f;
    state.cruise = band ? pick(rng, band->speed) : 2.0f;
    state.target = state.origin + Vec3 { std::cos(angle), std::sin(angle), 0.0f } * distance;
    state.hasTarget = true;
    state.stopAtTarget = chance(rng, script.stopFraction);
    state.brake = pick(rng, script.brake);
    state.atTip = true;
    markChange(state);
}

void pickCenter(const PilotScript3D& script, PilotState3D& state, std::mt19937_64& rng) {
    state.target = state.origin;
    state.hasTarget = true;
    if (chance(rng, 0.5f))
        if (const PilotBand3D* band = pickBand(script, rng)) state.cruise = pick(rng, band->speed);
    state.stopAtTarget = chance(rng, script.stopFraction);
    state.brake = pick(rng, script.brake);
    state.atTip = false;
    markChange(state);
}

// Um pedido novo do segurar: outra direcao em relacao a camera, outra faixa, ou solto.
void pickStick(const PilotScript3D& script, PilotState3D& state, const PilotView3D& view, std::mt19937_64& rng) {
    state.stickTime = pick(rng, script.stickSeconds);
    markChange(state);
    if (chance(rng, script.releaseFraction)) {
        state.stickForward = state.stickLeft = state.speed = 0.0f;
        return;
    }
    const PilotBand3D* band = pickBand(script, rng);
    state.speed = band ? pick(rng, band->speed) : 2.0f;
    const bool keys = chance(rng, script.keysFraction);
    float relative = 0.0f;
    const float awayX = view.position.x - state.origin.x, awayY = view.position.y - state.origin.y;
    if (std::hypot(awayX, awayY) > script.leashMeters) {
        // Longe da origem: uma direcao de volta (ate 60 graus de lado).
        const float home = std::atan2(-awayY, -awayX) + std::uniform_real_distribution<float>(-Pi / 3.0f, Pi / 3.0f)(rng);
        relative = wrap(home - state.camera);
    } else {
        relative = keys ? static_cast<float>(std::uniform_int_distribution<int>(0, 7)(rng)) * Pi / 4.0f
                        : std::uniform_real_distribution<float>(-Pi, Pi)(rng);
    }
    if (keys) relative = std::round(relative / (Pi / 4.0f)) * (Pi / 4.0f);
    state.stickForward = std::cos(relative);
    state.stickLeft = std::sin(relative);
    state.stickWorld = wrap(state.camera + relative);
    if (state.cameraMode == PilotCameraMode3D::Turns && state.cameraTurnLeft == 0.0f && chance(rng, script.syncFraction))
        startTurn(script, state, rng);
}

} // namespace

void CommandPilot3D::start(const PilotScript3D& script, PilotState3D& state, Vec3 origin, float heading,
    std::mt19937_64& rng) {
    state = {};
    state.origin = { origin.x, origin.y, 0.0f };
    state.startHeading = heading;
    state.camera = wrap(heading);
    const PilotCamera3D& c = script.camera;
    const float total = std::max(1e-6f, c.fixedWeight + c.followWeight + c.turnsWeight);
    const float roll = std::uniform_real_distribution<float>(0.0f, total)(rng);
    state.cameraMode = roll < c.fixedWeight ? PilotCameraMode3D::Fixed
        : roll < c.fixedWeight + c.followWeight ? PilotCameraMode3D::Follow
                                                : PilotCameraMode3D::Turns;
    state.cameraPause = pick(rng, c.pauseSeconds);
    const PilotView3D view { state.origin, {} };
    if (script.kind == "star") pickTip(script, state, rng, false);
    else if (script.kind == "stick") pickStick(script, state, view, rng);
    state.changed = false;
    state.changes = 0;
}

ControlInput3D CommandPilot3D::update(const PilotScript3D& script, PilotState3D& state, const PilotView3D& view,
    float dt, std::mt19937_64& rng) {
    state.clock += dt;
    state.changed = false;
    float worldDirection = state.camera;
    bool moving = false;
    if (script.kind == "star" && state.hasTarget) {
        const Vec3 to { state.target.x - view.position.x, state.target.y - view.position.y, 0.0f };
        const float distance = std::hypot(to.x, to.y);
        const float planar = std::hypot(view.velocity.x, view.velocity.y);
        state.speed = 0.0f;
        if (state.holdLeft > 0.0f) {
            // Parado no ponto; acabou, o seguinte.
            if ((state.holdLeft -= dt) <= 0.0f) {
                state.holdLeft = 0.0f;
                if (state.atTip && !chance(rng, script.continuousFraction)) pickCenter(script, state, rng);
                else pickTip(script, state, rng, state.atTip);
            }
        } else if (distance < script.arriveRadius) {
            if (!state.stopAtTarget) {
                ++state.reached;
                if (state.atTip && !chance(rng, script.continuousFraction)) pickCenter(script, state, rng);
                else pickTip(script, state, rng, state.atTip);
            } else if (planar < 0.4f) {
                ++state.reached;
                state.holdLeft = pick(rng, script.holdSeconds);
                markChange(state);
            }
        } else {
            worldDirection = std::atan2(to.y, to.x);
            state.speed = state.cruise;
            // Chegando num ponto de parar, ele vai soltando (a freada do piloto).
            if (state.stopAtTarget)
                state.speed = std::min(state.speed,
                    std::sqrt(2.0f * state.brake * std::max(0.0f, distance - 0.3f * script.arriveRadius)));
            moving = state.speed > 0.0f;
        }
    } else if (script.kind == "stick") {
        if ((state.stickTime -= dt) <= 0.0f) pickStick(script, state, view, rng);
        moving = state.speed > 0.0f;
        worldDirection = state.cameraMode == PilotCameraMode3D::Follow
            ? state.stickWorld : wrap(state.camera + std::atan2(state.stickLeft, state.stickForward));
    }

    // A camera.
    const float previous = state.camera;
    if (state.cameraMode == PilotCameraMode3D::Follow && moving) {
        const float error = wrap(worldDirection - state.camera);
        const float limit = script.camera.followRate * dt;
        state.camera = wrap(state.camera + std::clamp(error, -limit, limit));
    } else if (state.cameraMode == PilotCameraMode3D::Turns) {
        if (state.cameraTurnLeft != 0.0f) {
            const float step = std::min(std::abs(state.cameraTurnLeft), state.cameraTurnRate * dt);
            const float sign = state.cameraTurnLeft > 0.0f ? 1.0f : -1.0f;
            state.camera = wrap(state.camera + sign * step);
            state.cameraTurnLeft -= sign * step;
            if (std::abs(state.cameraTurnLeft) < 1.0e-4f) {
                state.cameraTurnLeft = 0.0f;
                state.cameraPause = pick(rng, script.camera.pauseSeconds);
            }
        } else if ((state.cameraPause -= dt) <= 0.0f) {
            startTurn(script, state, rng);
        }
    }

    ControlInput3D input;
    input.cameraHeading = state.camera;
    input.cameraRate = dt > 0.0f ? wrap(state.camera - previous) / dt : 0.0f;
    if (moving) {
        // A estrela e o segurar acompanhado seguram uma direcao no mundo (o
        // analogico corrigido a cada decisao); o resto, a direcao na camera.
        const bool world = script.kind == "star" || state.cameraMode == PilotCameraMode3D::Follow;
        const float relative = world ? wrap(worldDirection - state.camera) : std::atan2(state.stickLeft, state.stickForward);
        input.stickForward = std::cos(relative);
        input.stickLeft = std::sin(relative);
        input.speed = state.speed;
    }
    return input;
}

} // namespace MatterEngine
