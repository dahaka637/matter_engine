#include "Engine/RL/Commands/PilotScriptJson3D.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>

namespace MatterEngine {
namespace {

constexpr float Degrees = 3.14159265358979f / 180.0f;

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error("roteiro do piloto: " + what);
}

PilotRange3D range(const nlohmann::json& object, const char* key, PilotRange3D fallback, float scale = 1.0f) {
    if (!object.contains(key)) return fallback;
    const nlohmann::json& values = object.at(key);
    if (!values.is_array() || values.size() != 2) fail(std::string("faixa invalida em ") + key);
    const PilotRange3D result { values[0].get<float>() * scale, values[1].get<float>() * scale };
    if (!(result.minimum <= result.maximum)) fail(std::string("faixa invertida em ") + key);
    return result;
}

} // namespace

PilotScript3D pilotScriptFromJson3D(const nlohmann::json& p, const std::string& drill) {
    PilotScript3D script;
    script.kind = p.value("kind", script.kind);
    script.forwardOnly = p.value("forwardOnly", script.forwardOnly);
    if (script.kind != "hold" && script.kind != "stick" && script.kind != "star" && script.kind != "arc")
        fail("o piloto do exercicio " + drill + " tem kind desconhecido: " + script.kind);
    if (p.contains("bands"))
        for (const nlohmann::json& b : p["bands"]) {
            PilotBand3D band;
            band.name = b.value("name", std::string());
            band.speed = range(b, "speed", band.speed);
            band.distance = range(b, "distance", { 6.0f, 10.0f });
            band.weight = b.value("weight", 1.0f);
            if (!(band.speed.minimum > 0.0f) || band.weight < 0.0f) fail("faixa de velocidade invalida em " + drill);
            script.bands.push_back(band);
        }
    if (script.kind != "hold" && script.bands.empty()) fail("o exercicio " + drill + " anda sem faixas de velocidade");
    if (p.contains("camera")) {
        const nlohmann::json& c = p["camera"];
        PilotCamera3D& camera = script.camera;
        camera.fixedWeight = c.value("fixed", camera.fixedWeight);
        camera.followWeight = c.value("follow", camera.followWeight);
        camera.turnsWeight = c.value("turns", camera.turnsWeight);
        camera.followRate = c.value("followRate", camera.followRate);
        camera.turnRadians = range(c, "turnDegrees", { camera.turnRadians.minimum / Degrees,
            camera.turnRadians.maximum / Degrees }, Degrees);
        camera.turnRate = range(c, "turnRate", camera.turnRate);
        camera.pauseSeconds = range(c, "pauseSeconds", camera.pauseSeconds);
        if (!(camera.fixedWeight + camera.followWeight + camera.turnsWeight > 0.0f) || camera.fixedWeight < 0.0f
            || camera.followWeight < 0.0f || camera.turnsWeight < 0.0f)
            fail("os modos da camera sem peso em " + drill);
    }
    script.stickSeconds = range(p, "stickSeconds", script.stickSeconds);
    script.releaseFraction = p.value("releaseFraction", script.releaseFraction);
    script.keysFraction = p.value("keysFraction", script.keysFraction);
    script.syncFraction = p.value("syncFraction", script.syncFraction);
    script.leashMeters = p.value("leashMeters", script.leashMeters);
    if (p.contains("directions"))
        for (const nlohmann::json& d : p["directions"]) script.stickDirections.push_back(d.get<float>() * Degrees);
    script.starDirections = p.value("starDirections", script.starDirections);
    script.starJitterDegrees = p.value("starJitterDegrees", script.starJitterDegrees);
    script.continuousFraction = p.value("continuousFraction", script.continuousFraction);
    script.stopFraction = p.value("stopFraction", script.stopFraction);
    script.holdSeconds = range(p, "holdSeconds", script.holdSeconds);
    script.brake = range(p, "brake", script.brake);
    script.arriveRadius = p.value("arriveRadius", script.arriveRadius);
    script.arcRadius = range(p, "arcRadius", script.arcRadius);
    script.arcDegrees = range(p, "arcDegrees", script.arcDegrees);
    script.arcFlipFraction = p.value("arcFlipFraction", script.arcFlipFraction);
    script.lateralAcceleration = p.value("lateralAcceleration", script.lateralAcceleration);
    script.arcLookahead = p.value("arcLookahead", script.arcLookahead);
    // (Um arco ate 170 graus: o "quanto falta" e medido por um angulo entre -180 e 180.)
    if (!(script.arcRadius.minimum > 0.5f) || !(script.arcDegrees.minimum > 0.0f) || script.arcDegrees.maximum > 170.0f
        || !(script.lateralAcceleration > 0.0f)
        || !(script.arcLookahead > 0.0f))
        fail("curva invalida em " + drill);
    if (!(script.stickSeconds.minimum > 0.0f) || script.starDirections == 0 || !(script.arriveRadius > 0.0f)
        || !(script.brake.minimum > 0.0f))
        fail("roteiro invalido em " + drill);
    return script;
}


} // namespace MatterEngine
