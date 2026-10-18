#include "Engine/Athlete/Training/LocomotionTask3D.hpp"

#include "Engine/RL/Commands/PilotScriptJson3D.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace MatterEngine {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error("tarefa da locomocao: " + what);
}

void read(const nlohmann::json& object, const char* key, float& value) {
    if (object.contains(key)) value = object.at(key).get<float>();
}

} // namespace

LocomotionTask3D loadLocomotionTask3D(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("nao abriu " + path.string());
    const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-locomocao-1") fail("schema desconhecido em " + path.string());

    LocomotionTask3D task;
    task.id = document.at("id").get<std::string>();
    task.displayName = document.value("displayName", task.id);
    task.definitionPath = path;
    // (A impressao da tarefa: o texto inteiro.)
    std::uint64_t hash = 1469598103934665603ull;
    for (unsigned char c : text) {
        hash ^= c;
        hash *= 1099511628211ull;
    }
    task.hash = hash;
    read(document, "episodeSeconds", task.episodeSeconds);
    if (!(task.episodeSeconds > 0.0f)) fail("duracao invalida");
    if (document.contains("assist")) {
        const nlohmann::json& a = document["assist"];
        read(a, "standing", task.standingAssistPercent);
        read(a, "walking", task.walkingAssistPercent);
        read(a, "feet", task.feetAssistPercent);
        read(a, "getUp", task.getUpAssistPercent);
    }
    if (document.contains("turnMove")) {
        const nlohmann::json& m = document["turnMove"];
        task.turnMove.enabled = m.value("enabled", false);
        read(m, "dropout", task.turnMove.dropout);
        read(m, "amount", task.turnMove.amount);
        if (!(task.turnMove.dropout >= 0.0f && task.turnMove.dropout <= 1.0f)) fail("turnMove.dropout fora de 0..1");
    }
    if (document.contains("caps")) {
        const nlohmann::json& c = document["caps"];
        LocomotionCaps3D& caps = task.caps;
        read(c, "touchdownMeters", caps.touchdownMeters);
        read(c, "swingUpMeters", caps.swingUpMeters);
        read(c, "swingDownMeters", caps.swingDownMeters);
        read(c, "phaseLeadCycles", caps.phaseLeadCycles);
        read(c, "recoveryUrgency", caps.recoveryUrgency);
        read(c, "cadenceOctaves", caps.cadenceOctaves);
        read(c, "strideOctaves", caps.strideOctaves);
        read(c, "leanRadians", caps.leanRadians);
        read(c, "comMeters", caps.comMeters);
        read(c, "pelvisUpMeters", caps.pelvisUpMeters);
        read(c, "pelvisDownMeters", caps.pelvisDownMeters);
        read(c, "contactAcceleration", caps.contactAcceleration);
        read(c, "jointRadians", caps.jointRadians);
        read(c, "gainOctaves", caps.gainOctaves);
    }
    if (document.contains("reward")) {
        const nlohmann::json& r = document["reward"];
        LocomotionRewardWeights3D& w = task.reward;
        read(r, "velocity", w.velocity);
        read(r, "facing", w.facing);
        read(r, "style", w.style);
        read(r, "contact", w.contact);
        read(r, "velocitySigma", w.velocitySigma);
        read(r, "facingSigma", w.facingSigma);
        read(r, "styleSigma", w.styleSigma);
        read(r, "contactSigma", w.contactSigma);
        read(r, "slip", w.slip);
        read(r, "work", w.work);
        read(r, "smooth", w.smooth);
        read(r, "assist", w.assist);
        read(r, "posture", w.posture);
        read(r, "residual", w.residual);
        read(r, "still", w.still);
        read(r, "fall", w.fall);
        if (!(w.velocitySigma > 0.0f) || !(w.facingSigma > 0.0f) || !(w.styleSigma > 0.0f) || !(w.contactSigma > 0.0f))
            fail("sigma da nota invalido");
    }
    if (document.contains("termination")) {
        const nlohmann::json& t = document["termination"];
        read(t, "pelvisHeight", task.fallPelvisHeight);
        read(t, "boundaryMeters", task.boundaryMeters);
    }
    if (document.contains("drillWeights")) {
        const nlohmann::json& d = document["drillWeights"];
        task.drillWeights.automatic = d.value("auto", task.drillWeights.automatic);
        read(d, "gain", task.drillWeights.gain);
        read(d, "maximumFactor", task.drillWeights.maximumFactor);
        read(d, "floor", task.drillWeights.floor);
    }
    if (!document.contains("drills") || !document["drills"].is_array() || document["drills"].empty())
        fail("sem exercicios");
    for (const nlohmann::json& d : document["drills"]) {
        LocomotionDrill3D drill;
        drill.id = d.at("id").get<std::string>();
        drill.displayName = d.value("displayName", drill.id);
        drill.phase = d.value("phase", drill.phase);
        read(d, "weight", drill.weight);
        if (!(drill.weight >= 0.0f)) fail("peso invalido em " + drill.id);
        drill.pilot = pilotScriptFromJson3D(d.value("pilot", nlohmann::json::object()), drill.id);
        if (d.contains("minimum")) {
            read(d["minimum"], "fallFraction", drill.maximumFallFraction);
            read(d["minimum"], "velocityError", drill.maximumVelocityError);
        }
        for (const LocomotionDrill3D& other : task.drills)
            if (other.id == drill.id) fail("exercicio repetido: " + drill.id);
        task.drills.push_back(std::move(drill));
    }
    return task;
}

} // namespace MatterEngine
