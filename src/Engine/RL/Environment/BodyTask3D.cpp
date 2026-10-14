#include "Engine/RL/Environment/BodyTask3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace MatterEngine {
namespace {

constexpr float Degrees = 3.14159265358979f / 180.0f;

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error("tarefa do Corpo: " + what);
}

PilotRange3D range(const nlohmann::json& object, const char* key, PilotRange3D fallback, float scale = 1.0f) {
    if (!object.contains(key)) return fallback;
    const nlohmann::json& values = object.at(key);
    if (!values.is_array() || values.size() != 2) fail(std::string("faixa invalida em ") + key);
    const PilotRange3D result { values[0].get<float>() * scale, values[1].get<float>() * scale };
    if (!(result.minimum <= result.maximum)) fail(std::string("faixa invertida em ") + key);
    return result;
}

PilotScript3D parsePilot(const nlohmann::json& p, const std::string& drill) {
    PilotScript3D script;
    script.kind = p.value("kind", script.kind);
    if (script.kind != "hold" && script.kind != "stick" && script.kind != "star")
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
    script.starDirections = p.value("starDirections", script.starDirections);
    script.starJitterDegrees = p.value("starJitterDegrees", script.starJitterDegrees);
    script.continuousFraction = p.value("continuousFraction", script.continuousFraction);
    script.stopFraction = p.value("stopFraction", script.stopFraction);
    script.holdSeconds = range(p, "holdSeconds", script.holdSeconds);
    script.brake = range(p, "brake", script.brake);
    script.arriveRadius = p.value("arriveRadius", script.arriveRadius);
    if (!(script.stickSeconds.minimum > 0.0f) || script.starDirections == 0 || !(script.arriveRadius > 0.0f)
        || !(script.brake.minimum > 0.0f))
        fail("roteiro invalido em " + drill);
    return script;
}

} // namespace

BodyTask3D loadBodyTask3D(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("nao abriu " + path.string());
    const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-task-2") fail("schema desconhecido em " + path.string());

    BodyTask3D task;
    task.id = document.at("id").get<std::string>();
    task.displayName = document.value("displayName", task.id);
    task.definitionPath = path;
    task.bodyPath = (path.parent_path() / document.at("body").get<std::string>()).lexically_normal();
    task.episodeSeconds = document.value("episodeSeconds", task.episodeSeconds);
    task.groundMaterial = document.value("groundMaterial", task.groundMaterial);
    task.worldSpacingMeters = document.value("worldSpacingMeters", task.worldSpacingMeters);
    if (!(task.episodeSeconds > 0.0f) || !(task.worldSpacingMeters > 0.0f)) fail("duracao ou distancia invalidas");

    if (document.contains("decision")) {
        const nlohmann::json& d = document["decision"];
        BodyContractSettings3D& c = task.contract;
        c.substepsPerDecision = d.value("substeps", c.substepsPerDecision);
        c.historyDecisions = d.value("historyDecisions", c.historyDecisions);
        const std::string velocity = d.value("velocityTarget", std::string("mean_slope"));
        if (velocity == "zero") c.velocityTarget = BodyVelocityTarget3D::Zero;
        else if (velocity == "sampled_slope") c.velocityTarget = BodyVelocityTarget3D::SampledSlope;
        else if (velocity == "mean_slope") c.velocityTarget = BodyVelocityTarget3D::MeanSlope;
        else fail("velocityTarget desconhecido: " + velocity);
        c.velocityFraction = d.value("velocityFraction", c.velocityFraction);
        c.maximumTargetVelocity = d.value("maximumTargetVelocity", c.maximumTargetVelocity);
        if (c.substepsPerDecision == 0 || c.substepsPerDecision > 16 || c.historyDecisions > 8)
            fail("decisao invalida");
    }
    task.contract.decisionSeconds = static_cast<float>(task.contract.substepsPerDecision) / 120.0f;

    if (document.contains("player")) {
        const nlohmann::json& p = document["player"];
        PlayerAttributes3D& player = task.player;
        player.heightMeters = p.value("height", player.heightMeters);
        player.massKg = p.value("mass", player.massKg);
        if (p.contains("strength")) {
            const nlohmann::json& s = p["strength"];
            if (!s.is_array() || s.size() != 3) fail("player.strength precisa de 3 valores (pernas, tronco, bracos)");
            for (std::size_t i = 0; i < 3; ++i) player.strength[i] = s[i].get<float>();
        }
        player.sprintCap = p.value("sprintCap", player.sprintCap);
        player.capNow = player.sprintCap;
    }
    if (document.contains("speedCaps")) {
        const nlohmann::json& s = document["speedCaps"];
        SpeedCaps3D& caps = task.control.caps;
        caps.front = s.value("front", caps.front);
        caps.diagonalFront = s.value("diagonalFront", caps.diagonalFront);
        caps.side = s.value("side", caps.side);
        caps.diagonalBack = s.value("diagonalBack", caps.diagonalBack);
        caps.back = s.value("back", caps.back);
    }
    task.control.jumpWindowSeconds = document.value("jumpWindowSeconds", task.control.jumpWindowSeconds);

    if (document.contains("reward")) {
        const nlohmann::json& r = document["reward"];
        BodyRewardSettings3D& w = task.reward;
        w.velocityForm = r.value("velocityForm", w.velocityForm);
        if (w.velocityForm != "cauchy" && w.velocityForm != "bell") fail("velocityForm desconhecida: " + w.velocityForm);
        w.velocityToleranceAbsolute = r.value("velocityToleranceAbsolute", w.velocityToleranceAbsolute);
        w.velocityToleranceRelative = r.value("velocityToleranceRelative", w.velocityToleranceRelative);
        w.facingSigma = r.value("facingSigma", w.facingSigma);
        if (r.contains("costs")) {
            const nlohmann::json& c = r["costs"];
            w.work = c.value("work", w.work);
            w.torque = c.value("torque", w.torque);
            w.slip = c.value("slip", w.slip);
            w.impact = c.value("impact", w.impact);
            w.head = c.value("head", w.head);
            w.smooth = c.value("smooth", w.smooth);
            w.limits = c.value("limits", w.limits);
            w.contacts = c.value("contacts", w.contacts);
        }
        w.fall = r.value("fall", w.fall);
        for (const float v : { w.work, w.torque, w.slip, w.impact, w.head, w.smooth, w.limits, w.contacts, w.fall })
            if (v < 0.0f) fail("os custos e a queda nao podem ser negativos");
        if (!(w.velocityToleranceAbsolute > 0.0f) || !(w.facingSigma > 0.0f)) fail("tolerancias invalidas");
    }
    if (document.contains("termination")) {
        const nlohmann::json& t = document["termination"];
        auto& end = task.termination;
        end.minimumHeadHeightFraction = t.value("minimumHeadHeightFraction", end.minimumHeadHeightFraction);
        end.minimumPelvisHeightFraction = t.value("minimumPelvisHeightFraction", end.minimumPelvisHeightFraction);
        end.nonSupportContact = t.value("nonSupportContact", end.nonSupportContact);
        end.maximumDistanceFromStart = t.value("maximumDistanceFromStart", end.maximumDistanceFromStart);
    }
    if (document.contains("initialState")) {
        const nlohmann::json& s = document["initialState"];
        auto& start = task.initialState;
        start.posture = s.value("posture", start.posture);
        start.jointNoiseRadians = s.value("jointNoiseRadians", start.jointNoiseRadians);
        start.jointVelocityNoise = s.value("jointVelocityNoise", start.jointVelocityNoise);
        start.rootVelocityNoise = s.value("rootVelocityNoise", start.rootVelocityNoise);
        start.rootAngularVelocityNoise = s.value("rootAngularVelocityNoise", start.rootAngularVelocityNoise);
        start.randomYaw = s.value("randomYaw", start.randomYaw);
        start.referenceFraction = s.value("referenceFraction", start.referenceFraction);
        start.referenceMinimumPelvis = s.value("referenceMinimumPelvis", start.referenceMinimumPelvis);
        if (start.referenceFraction < 0.0f || start.referenceFraction > 1.0f) fail("referenceFraction fora de 0..1");
    }
    if (document.contains("style") && document["style"].contains("dataset"))
        task.styleDataset =
            (path.parent_path() / document["style"]["dataset"].get<std::string>()).lexically_normal();
    if (task.initialState.referenceFraction > 0.0f && task.styleDataset.empty())
        fail("os comecos pelo acervo pedem style.dataset");
    if (document.contains("randomization")) {
        const nlohmann::json& r = document["randomization"];
        auto& random = task.randomization;
        random.friction = range(r, "friction", random.friction);
        random.mass = range(r, "mass", random.mass);
        random.strength = range(r, "strength", random.strength);
        random.actionDelayProbability = r.value("actionDelayProbability", random.actionDelayProbability);
        random.observationNoise = r.value("observationNoise", random.observationNoise);
    }

    if (!document.contains("drills") || document["drills"].empty()) fail("sem exercicios");
    float total = 0.0f;
    for (const nlohmann::json& d : document["drills"]) {
        BodyDrill3D drill;
        drill.id = d.at("id").get<std::string>();
        drill.displayName = d.value("displayName", drill.id);
        drill.weight = d.value("weight", 1.0f);
        drill.episodeSeconds = d.value("episodeSeconds", 0.0f);
        drill.pilot = parsePilot(d.value("pilot", nlohmann::json::object()), drill.id);
        if (d.contains("task")) {
            drill.task.velocity = d["task"].value("velocity", drill.task.velocity);
            drill.task.facing = d["task"].value("facing", drill.task.facing);
        }
        if (drill.weight < 0.0f || drill.task.velocity < 0.0f || drill.task.facing < 0.0f
            || std::abs(drill.task.velocity + drill.task.facing - 1.0f) > 1.0e-3f)
            fail("os pesos do exercicio " + drill.id + " (a tarefa soma 1)");
        if (findBodyDrill3D(task, drill.id) >= 0) fail("exercicio repetido: " + drill.id);
        total += drill.weight;
        task.drills.push_back(std::move(drill));
    }
    if (!(total > 0.0f)) fail("exercicios sem peso");

    std::uint64_t hash = 14695981039346656037ull;
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    task.hash = hash;
    return task;
}

int findBodyDrill3D(const BodyTask3D& task, const std::string& id) {
    for (std::size_t d = 0; d < task.drills.size(); ++d)
        if (task.drills[d].id == id) return static_cast<int>(d);
    return -1;
}

} // namespace MatterEngine
