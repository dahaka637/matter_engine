#include "Engine/RL/Environment/TrainingTask3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace MatterEngine {
namespace {

TrainingRange3D range(const nlohmann::json& object, const char* key, TrainingRange3D fallback) {
    if (!object.contains(key)) return fallback;
    const auto& values = object.at(key);
    if (!values.is_array() || values.size() != 2) throw std::runtime_error(std::string("tarefa de RL: faixa invalida em ") + key);
    TrainingRange3D result { values[0].get<float>(), values[1].get<float>() };
    if (!(result.minimum <= result.maximum)) throw std::runtime_error(std::string("tarefa de RL: faixa invertida em ") + key);
    return result;
}

} // namespace

TrainingTask3D loadTrainingTask3D(const std::filesystem::path& definitionPath) {
    std::ifstream input(definitionPath, std::ios::binary);
    if (!input) throw std::runtime_error("tarefa de RL: arquivo ausente: " + definitionPath.string());
    const std::string text { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
    const nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-task-1")
        throw std::runtime_error("tarefa de RL: schema desconhecido em " + definitionPath.string());

    TrainingTask3D task;
    task.id = document.at("id").get<std::string>();
    task.displayName = document.value("displayName", task.id);
    task.definitionPath = definitionPath;
    task.bodyPath = (definitionPath.parent_path() / document.at("body").get<std::string>()).lexically_normal();
    task.actionRepeat = document.value("actionRepeat", 4u);
    task.episodeSeconds = document.value("episodeSeconds", 20.0f);
    task.groundMaterial = document.value("groundMaterial", std::string("grass"));
    if (task.actionRepeat == 0 || task.actionRepeat > 16 || !(task.episodeSeconds > 0.0f))
        throw std::runtime_error("tarefa de RL: actionRepeat/episodeSeconds invalidos");
    if (document.contains("postures")) {
        task.postures.clear();
        for (const nlohmann::json& entry : document["postures"])
            task.postures.push_back(
                { entry.at("id").get<std::string>(), entry.value("weight", 1.0f), entry.value("standingOnly", false) });
        float total = 0.0f;
        for (const TrainingPostureWeight3D& posture : task.postures) total += std::max(0.0f, posture.weight);
        if (task.postures.empty() || !(total > 0.0f)) throw std::runtime_error("tarefa de RL: posturas sem peso");
    }

    if (document.contains("reward")) {
        const nlohmann::json& r = document["reward"];
        TrainingRewardWeights3D& w = task.reward;
        w.alive = r.value("alive", w.alive);
        w.upright = r.value("upright", w.upright);
        w.headHeight = r.value("headHeight", w.headHeight);
        w.comOverSupport = r.value("comOverSupport", w.comOverSupport);
        w.still = r.value("still", w.still);
        w.trunkTurn = r.value("trunkTurn", w.trunkTurn);
        w.footSlip = r.value("footSlip", w.footSlip);
        w.footDrift = r.value("footDrift", w.footDrift);
        w.effort = r.value("effort", w.effort);
        w.actionRate = r.value("actionRate", w.actionRate);
        w.jointLimit = r.value("jointLimit", w.jointLimit);
        w.posePrior = r.value("posePrior", w.posePrior);
        w.poseMatch = r.value("poseMatch", w.poseMatch);
        w.velocity = r.value("velocity", w.velocity);
        w.yawRate = r.value("yawRate", w.yawRate);
        w.feetAirTime = r.value("feetAirTime", w.feetAirTime);
        w.gaitContact = r.value("gaitContact", w.gaitContact);
        w.swingHeight = r.value("swingHeight", w.swingHeight);
        w.gaitReference = r.value("gaitReference", w.gaitReference);
        w.fall = r.value("fall", w.fall);
        w.positiveOnly = r.value("positiveOnly", w.positiveOnly);
    }
    task.worldSpacingMeters = document.value("worldSpacingMeters", task.worldSpacingMeters);
    if (!(task.worldSpacingMeters >= 4.0f)) throw std::runtime_error("tarefa de RL: worldSpacingMeters invalido");
    if (document.contains("commands")) {
        const nlohmann::json& c = document["commands"];
        auto& out = task.commands;
        out.resampleSeconds = range(c, "resampleSeconds", out.resampleSeconds);
        out.standingFraction = std::clamp(c.value("standingFraction", out.standingFraction), 0.0f, 1.0f);
        out.forward = std::max(0.0f, c.value("forward", out.forward));
        out.backward = std::max(0.0f, c.value("backward", out.backward));
        out.lateral = std::max(0.0f, c.value("lateral", out.lateral));
        out.yawRate = std::max(0.0f, c.value("yawRate", out.yawRate));
        out.gateByTracking = c.value("gateByTracking", out.gateByTracking);
        out.startMoving = c.value("startMoving", out.startMoving);
        out.relativeToStanding = c.value("relativeToStanding", out.relativeToStanding);
        if (c.contains("limits")) {
            const nlohmann::json& l = c["limits"];
            out.limits.forward = l.value("forward", out.limits.forward);
            out.limits.backward = l.value("backward", out.limits.backward);
            out.limits.lateral = l.value("lateral", out.limits.lateral);
            out.limits.yawRate = l.value("yawRate", out.limits.yawRate);
        }
    }
    task.hasTeacher = document.contains("teacher") && document["teacher"].is_object();
    if (document.contains("gait")) {
        const nlohmann::json& g = document["gait"];
        auto& out = task.gait;
        if (g.contains("frequency")) {
            const auto& values = g["frequency"];
            if (!values.is_array() || values.size() != 2) throw std::runtime_error("tarefa de RL: gait.frequency invalida");
            out.frequency = { values[0].get<float>(), values[1].get<float>() };
        }
        out.doubleSupport = g.value("doubleSupport", out.doubleSupport);
        out.flightSpeed = std::max(0.1f, g.value("flightSpeed", out.flightSpeed));
        out.swingHeight = std::max(0.01f, g.value("swingHeight", out.swingHeight));
        if (g.contains("reference")) {
            const nlohmann::json& reference = g["reference"];
            out.referenceHipFlexion = std::max(0.0f, reference.value("hipFlexion", 0.0f));
            out.referenceFade = range(reference, "fade", out.referenceFade);
            if (!(out.referenceFade.maximum > out.referenceFade.minimum))
                throw std::runtime_error("tarefa de RL: gait.reference.fade invalido");
            for (const nlohmann::json& joint : reference.value("joints", nlohmann::json::array())) {
                const std::string leg = joint.value("leg", std::string());
                if (leg != "left" && leg != "right")
                    throw std::runtime_error("tarefa de RL: gait.reference.joints com perna invalida");
                out.referenceJoints.push_back(
                    { joint.at("dof").get<std::string>(), leg == "left" ? 0u : 1u, joint.value("scale", 0.0f) });
            }
        }
    }
    if (document.contains("termination")) {
        const nlohmann::json& t = document["termination"];
        auto& out = task.termination;
        out.minimumHeadHeightFraction = t.value("minimumHeadHeightFraction", out.minimumHeadHeightFraction);
        out.minimumPelvisHeightFraction = t.value("minimumPelvisHeightFraction", out.minimumPelvisHeightFraction);
        out.nonSupportContact = t.value("nonSupportContact", out.nonSupportContact);
        out.maximumDistanceFromStart = t.value("maximumDistanceFromStart", out.maximumDistanceFromStart);
    }
    if (document.contains("initialState")) {
        const nlohmann::json& s = document["initialState"];
        auto& out = task.initialState;
        out.jointNoiseRadians = s.value("jointNoiseRadians", out.jointNoiseRadians);
        out.jointVelocityNoise = s.value("jointVelocityNoise", out.jointVelocityNoise);
        out.rootVelocityNoise = s.value("rootVelocityNoise", out.rootVelocityNoise);
        out.rootAngularVelocityNoise = s.value("rootAngularVelocityNoise", out.rootAngularVelocityNoise);
        out.randomYaw = s.value("randomYaw", out.randomYaw);
    }
    if (document.contains("randomization")) {
        const nlohmann::json& r = document["randomization"];
        auto& out = task.randomization;
        out.friction = range(r, "friction", out.friction);
        out.mass = range(r, "mass", out.mass);
        out.strength = range(r, "strength", out.strength);
        out.actionDelayProbability = r.value("actionDelayProbability", out.actionDelayProbability);
        out.observationNoise = r.value("observationNoise", out.observationNoise);
    }
    if (document.contains("pushes")) {
        const nlohmann::json& p = document["pushes"];
        auto& out = task.pushes;
        out.enabled = p.value("enabled", out.enabled);
        out.intervalSeconds = range(p, "intervalSeconds", out.intervalSeconds);
        out.durationSeconds = range(p, "durationSeconds", out.durationSeconds);
        out.forceNewtons = range(p, "forceNewtons", out.forceNewtons);
        if (p.contains("links")) out.links = p["links"].get<std::vector<std::string>>();
    }

    std::uint64_t hash = 14695981039346656037ull;
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    task.hash = hash;
    return task;
}

} // namespace MatterEngine
