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

// A velocidade pedida de uma faixa, cortada pela janela de agora.
float pickSpeed(const PilotScript3D& script, const PilotBand3D* band, float fallback, std::mt19937_64& rng) {
    PilotRange3D range = band ? band->speed : PilotRange3D { fallback, fallback };
    const PilotRange3D& window = script.speedWindow;
    if (window.maximum > 0.0f) {
        const float low = std::max(range.minimum, window.minimum);
        const float high = std::min(range.maximum, window.maximum);
        range = low <= high ? PilotRange3D { low, high } : window;
    }
    return pick(rng, range);
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
    state.cruise = pickSpeed(script, band, 2.0f, rng);
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
        if (const PilotBand3D* band = pickBand(script, rng)) state.cruise = pickSpeed(script, band, 2.0f, rng);
    state.stopAtTarget = chance(rng, script.stopFraction);
    state.brake = pick(rng, script.brake);
    state.atTip = false;
    markChange(state);
}

// Um arco novo que comeca em from, andando no rumo heading (rad): o sentido
// (trocando as vezes), o raio, o tamanho, a velocidade (limitada pela curva) e
// se para no fim.
void pickArc(const PilotScript3D& script, PilotState3D& state, Vec3 from, float heading, std::mt19937_64& rng) {
    if (state.hasArc && chance(rng, script.arcFlipFraction)) state.arcSign = -state.arcSign;
    else if (!state.hasArc) state.arcSign = chance(rng, 0.5f) ? 1.0f : -1.0f;
    state.arcRadius = pick(rng, script.arcRadius);
    const Vec3 left { -std::sin(heading), std::cos(heading), 0.0f };
    state.arcCenter = Vec3 { from.x, from.y, 0.0f } + left * (state.arcSign * state.arcRadius);
    const float start = std::atan2(from.y - state.arcCenter.y, from.x - state.arcCenter.x);
    state.arcEnd = start + state.arcSign * pick(rng, script.arcDegrees) * Pi / 180.0f;
    state.target = state.arcCenter + Vec3 { std::cos(state.arcEnd), std::sin(state.arcEnd), 0.0f } * state.arcRadius;
    state.hasTarget = true;
    state.hasArc = true;
    state.cruise = std::min(pickSpeed(script, pickBand(script, rng), 1.5f, rng),
        std::sqrt(script.lateralAcceleration * state.arcRadius));
    state.stopAtTarget = chance(rng, script.stopFraction);
    state.brake = pick(rng, script.brake);
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
    state.speed = pickSpeed(script, band, 2.0f, rng);
    const bool keys = chance(rng, script.keysFraction);
    float relative = 0.0f;
    const float awayX = view.position.x - state.origin.x, awayY = view.position.y - state.origin.y;
    if (script.forwardOnly) {
        relative = 0.0f;
    } else if (!script.stickDirections.empty()) {
        relative = script.stickDirections[std::uniform_int_distribution<std::size_t>(0, script.stickDirections.size() - 1)(rng)];
    } else if (std::hypot(awayX, awayY) > script.leashMeters) {
        // Longe da origem: uma direcao de volta (ate 60 graus de lado).
        const float home = std::atan2(-awayY, -awayX) + std::uniform_real_distribution<float>(-Pi / 3.0f, Pi / 3.0f)(rng);
        relative = wrap(home - state.camera);
    } else {
        relative = keys ? static_cast<float>(std::uniform_int_distribution<int>(0, 7)(rng)) * Pi / 4.0f
                        : std::uniform_real_distribution<float>(-Pi, Pi)(rng);
    }
    if (keys && !script.forwardOnly && script.stickDirections.empty())
        relative = std::round(relative / (Pi / 4.0f)) * (Pi / 4.0f);
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
    else if (script.kind == "arc") pickArc(script, state, state.origin, heading, rng);
    state.changed = false;
    state.changes = 0;
}

void CommandPilot3D::follow(const PilotScript3D& script, PilotState3D& state, Vec3 velocity, std::mt19937_64& rng) {
    float speed = std::hypot(velocity.x, velocity.y);
    if (speed < 0.3f) return;
    if (script.speedWindow.maximum > 0.0f)
        speed = std::clamp(speed, script.speedWindow.minimum, script.speedWindow.maximum);
    const float direction = std::atan2(velocity.y, velocity.x);
    if (script.kind == "stick") {
        // So para a frente: o quadro do clipe da a velocidade, nao a direcao.
        const float relative = script.forwardOnly ? 0.0f : wrap(direction - state.camera);
        state.stickForward = std::cos(relative);
        state.stickLeft = std::sin(relative);
        state.stickWorld = wrap(state.camera + relative);
        state.speed = speed;
        state.stickTime = pick(rng, script.stickSeconds);
    } else if (script.kind == "star" && !script.bands.empty()) {
        const std::uint32_t directions = std::max(1u, script.starDirections);
        const float step = 2.0f * Pi / static_cast<float>(directions);
        const auto slot = static_cast<std::uint32_t>(
            (static_cast<int>(std::lround(wrap(direction - state.startHeading) / step)) + static_cast<int>(directions))
            % static_cast<int>(directions));
        // A faixa da velocidade de agora (ou a mais perto dela).
        const PilotBand3D* band = &script.bands.front();
        float closest = 1e9f;
        for (const PilotBand3D& candidate : script.bands) {
            const float gap = speed < candidate.speed.minimum ? candidate.speed.minimum - speed
                : speed > candidate.speed.maximum ? speed - candidate.speed.maximum : 0.0f;
            if (gap < closest) {
                closest = gap;
                band = &candidate;
            }
        }
        const float angle = state.startHeading + step * static_cast<float>(slot);
        state.lastSlot = slot;
        state.cruise = speed;
        state.target = state.origin + Vec3 { std::cos(angle), std::sin(angle), 0.0f } * pick(rng, band->distance);
        state.hasTarget = true;
        state.atTip = true;
    }
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
            // Parado no ponto (o quanto ele sai dele); acabou, o seguinte.
            state.holdDistanceSum += distance;
            ++state.holdDecisions;
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
    } else if (script.kind == "arc" && state.hasArc) {
        const Vec3 away { view.position.x - state.arcCenter.x, view.position.y - state.arcCenter.y, 0.0f };
        const float radial = std::hypot(away.x, away.y);
        const float angle = std::atan2(away.y, away.x);
        // O que falta do arco (rad, no sentido dele) e onde a mira fica.
        const float remaining = state.arcSign * wrap(state.arcEnd - angle);
        state.speed = 0.0f;
        if (state.holdLeft > 0.0f) {
            const float distance = std::hypot(state.target.x - view.position.x, state.target.y - view.position.y);
            state.holdDistanceSum += distance;
            ++state.holdDecisions;
            if ((state.holdLeft -= dt) <= 0.0f) {
                state.holdLeft = 0.0f;
                pickArc(script, state, state.target, wrap(state.arcEnd + state.arcSign * Pi / 2.0f), rng);
            }
        } else if (remaining * state.arcRadius < script.arriveRadius) {
            const float planar = std::hypot(view.velocity.x, view.velocity.y);
            if (!state.stopAtTarget) {
                ++state.reached;
                pickArc(script, state, state.target, wrap(state.arcEnd + state.arcSign * Pi / 2.0f), rng);
            } else if (planar < 0.4f) {
                ++state.reached;
                state.holdLeft = pick(rng, script.holdSeconds);
                markChange(state);
            } else {
                // Passou do ponto sem parar: volta a ele.
                worldDirection = std::atan2(state.target.y - view.position.y, state.target.x - view.position.x);
                state.speed = std::min(state.cruise, 0.8f);
                moving = true;
            }
        } else {
            state.arcErrorSum += std::abs(radial - state.arcRadius);
            ++state.arcDecisions;
            const float aim = angle + state.arcSign * std::min(remaining, script.arcLookahead / state.arcRadius);
            const Vec3 carrot = state.arcCenter + Vec3 { std::cos(aim), std::sin(aim), 0.0f } * state.arcRadius;
            worldDirection = std::atan2(carrot.y - view.position.y, carrot.x - view.position.x);
            state.speed = state.cruise;
            if (state.stopAtTarget)
                state.speed = std::min(state.speed,
                    std::sqrt(2.0f * state.brake * std::max(0.0f, remaining * state.arcRadius - 0.3f * script.arriveRadius)));
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
        const bool world = script.kind == "star" || script.kind == "arc" || state.cameraMode == PilotCameraMode3D::Follow;
        const float relative = world ? wrap(worldDirection - state.camera) : std::atan2(state.stickLeft, state.stickForward);
        input.stickForward = std::cos(relative);
        input.stickLeft = std::sin(relative);
        input.speed = state.speed;
    }
    return input;
}

} // namespace MatterEngine
