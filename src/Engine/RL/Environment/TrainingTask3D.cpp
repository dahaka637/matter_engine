#include "Engine/RL/Environment/TrainingTask3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
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

namespace {

TrainingTask3D parseTask(const nlohmann::json& document, const std::filesystem::path& definitionPath) {
    TrainingTask3D task;
    task.id = document.at("id").get<std::string>();
    task.displayName = document.value("displayName", task.id);
    task.listed = document.value("menu", true);
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
        w.imitationPose = r.value("imitationPose", w.imitationPose);
        w.imitationVelocity = r.value("imitationVelocity", w.imitationVelocity);
        w.imitationKeyBodies = r.value("imitationKeyBodies", w.imitationKeyBodies);
        w.imitationRoot = r.value("imitationRoot", w.imitationRoot);
        w.targetSpeed = r.value("targetSpeed", w.targetSpeed);
        w.targetReached = r.value("targetReached", w.targetReached);
        w.targetTurn = r.value("targetTurn", w.targetTurn);
        w.facingHold = r.value("facingHold", w.facingHold);
        w.speedMatch = r.value("speedMatch", w.speedMatch);
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
        if (c.contains("startMoving")) {
            const nlohmann::json& start = c["startMoving"];
            out.startMoving = start.is_boolean() ? (start.get<bool>() ? 1.0f : 0.0f)
                                                 : std::clamp(start.get<float>(), 0.0f, 1.0f);
        }
        out.relativeToStanding = c.value("relativeToStanding", out.relativeToStanding);
        if (c.contains("limits")) {
            const nlohmann::json& l = c["limits"];
            out.limits.forward = l.value("forward", out.limits.forward);
            out.limits.backward = l.value("backward", out.limits.backward);
            out.limits.lateral = l.value("lateral", out.limits.lateral);
            out.limits.yawRate = l.value("yawRate", out.limits.yawRate);
        }
    }
    task.hasTeacher = document.contains("teacher") && document["teacher"].is_object()
        && document["teacher"].value("enabled", true);
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
    if (document.contains("imitation") && document["imitation"].is_object()
        && document["imitation"].value("enabled", true)) {
        const nlohmann::json& m = document["imitation"];
        auto& out = task.imitation;
        out.enabled = true;
        for (const nlohmann::json& clip : m.at("clips")) {
            const std::string file = clip.at("clip").get<std::string>();
            out.clips.push_back({ clip.value("id", std::string()), (definitionPath.parent_path() / file).lexically_normal() });
        }
        if (out.clips.empty()) throw std::runtime_error("tarefa de RL: imitation sem clipes");
        out.standingStart = std::clamp(m.value("standingStart", out.standingStart), 0.0f, 1.0f);
        out.residualActions = m.value("residualActions", out.residualActions);
        out.library = m.value("library", out.library);
        if (m.contains("planner")) {
            const nlohmann::json& p = m["planner"];
            auto& planner = out.planner;
            planner.hysteresisDegrees = std::max(0.0f, p.value("hysteresisDegrees", planner.hysteresisDegrees));
            planner.minimumSeconds = std::max(0.0f, p.value("minimumSeconds", planner.minimumSeconds));
            if (p.contains("speedByDistance"))
                planner.speedByDistance = p["speedByDistance"].get<std::vector<std::array<float, 2>>>();
            for (std::size_t i = 0; i < planner.speedByDistance.size(); ++i)
                if (!(planner.speedByDistance[i][1] > 0.0f)
                    || (i > 0 && !(planner.speedByDistance[i][0] > planner.speedByDistance[i - 1][0])))
                    throw std::runtime_error("tarefa de RL: speedByDistance em ordem de distancia, velocidades acima de 0");
            planner.speedWeight = std::max(0.0f, p.value("speedWeight", planner.speedWeight));
            planner.directionWeight = std::max(0.0f, p.value("directionWeight", planner.directionWeight));
            planner.directionWeightFacing = p.value("directionWeightFacing", planner.directionWeightFacing);
            planner.speedFromCommand = p.value("speedFromCommand", planner.speedFromCommand);
            planner.freeBackwardDistance = std::max(0.0f, p.value("freeBackwardDistance", planner.freeBackwardDistance));
            planner.idleBelow = std::max(0.0f, p.value("idleBelow", planner.idleBelow));
        }
        if (m.contains("scales")) {
            const nlohmann::json& k = m["scales"];
            out.poseScale = std::max(0.0f, k.value("pose", out.poseScale));
            out.velocityScale = std::max(0.0f, k.value("velocity", out.velocityScale));
            out.keyBodyScale = std::max(0.0f, k.value("keyBodies", out.keyBodyScale));
            out.rootHeightScale = std::max(0.0f, k.value("rootHeight", out.rootHeightScale));
            out.rootVelocityScale = std::max(0.0f, k.value("rootVelocity", out.rootVelocityScale));
        }
    }
    if (document.contains("target") && document["target"].is_object() && document["target"].value("enabled", true)) {
        const nlohmann::json& t = document["target"];
        auto& out = task.target;
        out.enabled = true;
        out.distance = t.value("distance", out.distance);
        out.radius = t.value("radius", out.radius);
        out.firstAngleDegrees = std::max(0.0f, t.value("firstAngleDegrees", out.firstAngleDegrees));
        out.nextAngleDegrees = std::max(0.0f, t.value("nextAngleDegrees", out.nextAngleDegrees));
        out.nextAngleOffsetDegrees = t.value("nextAngleOffsetDegrees", out.nextAngleOffsetDegrees);
        if (t.contains("evaluationAngles")) out.evaluationAngles = t["evaluationAngles"].get<std::vector<float>>();
        out.referenceSpeed = t.value("referenceSpeed", out.referenceSpeed);
        out.speedCap = t.value("speedCap", out.speedCap);
        out.missMeters = std::max(0.0f, t.value("missMeters", out.missMeters));
        out.distanceRange = range(t, "distanceRange", out.distanceRange);
        out.evaluationPerWorld = t.value("evaluationPerWorld", out.evaluationPerWorld);
        if (t.contains("evaluationDistances")) out.evaluationDistances = t["evaluationDistances"].get<std::vector<float>>();
        out.turnGain = std::max(0.0f, t.value("turnGain", out.turnGain));
        out.turnMaximum = std::max(0.0f, t.value("turnMaximum", out.turnMaximum));
        out.turnSigma = std::max(1.0e-3f, t.value("turnSigma", out.turnSigma));
        out.facing = t.value("facing", out.facing);
        out.facingDriftRadians = std::max(0.0f, t.value("facingDriftRadians", out.facingDriftRadians));
        out.facingLink = t.value("facingLink", out.facingLink);
        if (out.facingLink != "pelvis" && out.facingLink != "chest")
            throw std::runtime_error("tarefa de RL: facingLink e pelvis ou chest");
        out.facingFraction = std::clamp(t.value("facingFraction", out.facingFraction), 0.0f, 1.0f);
        out.facingSigma = std::max(1.0e-3f, t.value("facingSigma", out.facingSigma));
        out.facingDriftFraction = std::clamp(t.value("facingDriftFraction", out.facingDriftFraction), 0.0f, 1.0f);
        out.facingDriftTurns = range(t, "facingDriftTurns", out.facingDriftTurns);
        out.freeLook = t.value("freeLook", out.freeLook);
        out.freeLookNext = std::clamp(t.value("freeLookNext", out.freeLookNext), 0.0f, 2.0f);
        out.facingDriftSpeed = range(t, "facingDriftSpeed", out.facingDriftSpeed);
        if (t.contains("speedCommand") && t["speedCommand"].is_object()) {
            const nlohmann::json& c = t["speedCommand"];
            auto& sc = out.speedCommand;
            sc.enabled = c.value("enabled", true);
            sc.bands.clear();
            for (const nlohmann::json& b : c.value("bands", nlohmann::json::array())) {
                TrainingTask3D::Target::SpeedBand band;
                band.name = b.value("name", std::string());
                band.speed = range(b, "speed", band.speed);
                band.distance = range(b, "distance", band.distance);
                band.weight = std::max(0.0f, b.value("weight", 1.0f));
                if (!(band.speed.minimum > 0.0f)) throw std::runtime_error("tarefa de RL: faixa de velocidade sem velocidade: " + band.name);
                sc.bands.push_back(band);
            }
            sc.brakeAcceleration = std::max(0.1f, c.value("brakeAcceleration", sc.brakeAcceleration));
            sc.minimumArrivalSpeed = std::max(0.0f, c.value("minimumArrivalSpeed", sc.minimumArrivalSpeed));
            sc.sigma = std::max(0.05f, c.value("sigma", sc.sigma));
            sc.redrawOnReturn = std::clamp(c.value("redrawOnReturn", sc.redrawOnReturn), 0.0f, 1.0f);
            sc.capByDirection = c.value("capByDirection", sc.capByDirection);
            sc.changesPerLeg = std::min(4u, c.value("changesPerLeg", sc.changesPerLeg));
            sc.stopFraction = std::clamp(c.value("stopFraction", sc.stopFraction), 0.0f, 1.0f);
            sc.stopHoldSeconds = range(c, "stopHoldSeconds", sc.stopHoldSeconds);
            sc.stopSpeed = std::max(0.05f, c.value("stopSpeed", sc.stopSpeed));
            if (sc.enabled && sc.bands.empty()) throw std::runtime_error("tarefa de RL: comando de velocidade sem faixas");
        }
        out.pattern = t.value("pattern", out.pattern);
        if (out.pattern != "random" && out.pattern != "star" && out.pattern != "arc")
            throw std::runtime_error("tarefa de RL: pattern e random, star ou arc");
        if (t.contains("arcRadii")) {
            out.arcRadii.clear();
            for (const nlohmann::json& r : t["arcRadii"]) {
                const auto pair = r.get<std::array<float, 2>>();
                out.arcRadii.push_back({ std::max(0.5f, pair[0]), std::max(0.5f, pair[1]) });
            }
        }
        out.arcSweepDegrees = range(t, "arcSweepDegrees", out.arcSweepDegrees);
        out.arcSpacing = std::max(0.5f, t.value("arcSpacing", out.arcSpacing));
        out.arcLateralAcceleration = std::max(0.5f, t.value("arcLateralAcceleration", out.arcLateralAcceleration));
        out.arcSwitchFraction = std::clamp(t.value("arcSwitchFraction", out.arcSwitchFraction), 0.0f, 1.0f);
        if (out.pattern == "arc" && out.arcRadii.empty()) throw std::runtime_error("tarefa de RL: curvas sem arcRadii");
        out.starDirections = std::clamp(t.value("starDirections", out.starDirections), 1u, 16u);
        out.starJitterDegrees = std::max(0.0f, t.value("starJitterDegrees", out.starJitterDegrees));
        out.lookahead = std::min(3u, t.value("lookahead", out.lookahead));
        out.racingLine = std::clamp(t.value("racingLine", out.racingLine), 0.0f, 0.95f);
        if (t.contains("evaluationCourses"))
            for (const nlohmann::json& c : t["evaluationCourses"]) {
                TrainingTask3D::Target::Course course;
                course.name = c.value("name", std::string());
                course.points = c.at("points").get<std::vector<std::array<float, 2>>>();
                if (course.points.empty()) throw std::runtime_error("tarefa de RL: percurso " + course.name + " vazio");
                // Cada ponto longe do anterior (e o primeiro do comeco): senao
                // chegar num ja seria chegar no seguinte.
                std::array<float, 2> previous { 0.0f, 0.0f };
                for (const auto& point : course.points) {
                    if (!(std::hypot(point[0] - previous[0], point[1] - previous[1]) > out.radius))
                        throw std::runtime_error("tarefa de RL: percurso " + course.name + " com pontos colados");
                    previous = point;
                }
                out.evaluationCourses.push_back(std::move(course));
            }
        if (!(out.distance > out.radius) || !(out.radius > 0.0f) || !(out.referenceSpeed > 0.0f)
            || out.evaluationAngles.empty())
            throw std::runtime_error("tarefa de RL: target invalido");
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

    for (const nlohmann::json& entry : document.value("evaluation", nlohmann::json::array()))
        if (entry.is_object() && entry.value("enabled", true) && entry.contains("name"))
            task.evaluationBatteries.push_back(entry["name"].get<std::string>());
    return task;
}

// O que um exercicio nao pode mudar: o que forma a observacao e a acao (a
// mesma rede serve a todos) e o mundo (o corpo, o chao, as posturas).
void requireSameLayout(const TrainingTask3D& base, const TrainingTask3D& drill, const std::string& id) {
    bool clips = base.imitation.clips.size() == drill.imitation.clips.size();
    for (std::size_t i = 0; clips && i < base.imitation.clips.size(); ++i)
        clips = base.imitation.clips[i].id == drill.imitation.clips[i].id
            && base.imitation.clips[i].path == drill.imitation.clips[i].path;
    const bool same = clips && base.bodyPath == drill.bodyPath && base.actionRepeat == drill.actionRepeat
        && base.groundMaterial == drill.groundMaterial && base.worldSpacingMeters == drill.worldSpacingMeters
        && base.imitation.enabled == drill.imitation.enabled && base.imitation.library == drill.imitation.library
        && base.imitation.residualActions == drill.imitation.residualActions
        && base.target.enabled == drill.target.enabled && base.target.lookahead == drill.target.lookahead
        && base.target.facing == drill.target.facing && base.postures.size() == drill.postures.size()
        && base.target.speedCommand.enabled == drill.target.speedCommand.enabled;
    if (!same)
        throw std::runtime_error("tarefa de RL: o exercicio " + id
            + " muda o que a rede ve ou faz (clipes, corpo, alvo, lookahead, facing, posturas)");
}

} // namespace

TrainingTask3D loadTrainingTask3D(const std::filesystem::path& definitionPath) {
    std::ifstream input(definitionPath, std::ios::binary);
    if (!input) throw std::runtime_error("tarefa de RL: arquivo ausente: " + definitionPath.string());
    const std::string text { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
    nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-task-1")
        throw std::runtime_error("tarefa de RL: schema desconhecido em " + definitionPath.string());

    const nlohmann::json drills = document.contains("drills") ? document["drills"] : nlohmann::json::array();
    document.erase("drills");
    TrainingTask3D task = parseTask(document, definitionPath);
    // Os exercicios: cada um e a tarefa base com o que ele muda por cima (o
    // merge patch do JSON: objetos se misturam, listas e valores se trocam).
    float total = 0.0f;
    for (const nlohmann::json& entry : drills) {
        TrainingTask3D::Drill drill;
        drill.id = entry.at("id").get<std::string>();
        drill.name = entry.value("name", std::string());
        if (drill.name.empty()) {
            drill.name = drill.id;
            if (!drill.name.empty()) drill.name[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(drill.name[0])));
        }
        drill.weight = std::max(0.0f, entry.value("weight", 1.0f));
        nlohmann::json merged = document;
        if (entry.contains("overrides")) merged.merge_patch(entry["overrides"]);
        drill.task = std::make_shared<TrainingTask3D>(parseTask(merged, definitionPath));
        requireSameLayout(task, *drill.task, drill.id);
        for (const TrainingTask3D::Drill& other : task.drills)
            if (other.id == drill.id) throw std::runtime_error("tarefa de RL: exercicio repetido: " + drill.id);
        total += drill.weight;
        task.drills.push_back(std::move(drill));
    }
    if (!task.drills.empty() && !(total > 0.0f)) throw std::runtime_error("tarefa de RL: exercicios sem peso");

    std::uint64_t hash = 14695981039346656037ull;
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    task.hash = hash;
    return task;
}

} // namespace MatterEngine
