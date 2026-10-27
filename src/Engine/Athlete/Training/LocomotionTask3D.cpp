#include "Engine/Athlete/Training/LocomotionTask3D.hpp"

#include "Engine/RL/Commands/PilotScriptJson3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
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
        read(a, "getUpStiffness", task.getUpStiffnessPercent);
        read(a, "getUpStrength", task.getUpStrengthPercent);
    }
    if (document.contains("assistRuler")) {
        const std::string ruler = document["assistRuler"].get<std::string>();
        if (ruler != "historical" && ruler != "linear") fail("assistRuler: historical ou linear");
        task.historicalAssistRuler = ruler == "historical";
    }
    if (document.contains("pushes")) {
        const nlohmann::json& p = document["pushes"];
        LocomotionPushes3D& q = task.pushes;
        read(p, "minimumNewtons", q.minimumNewtons);
        read(p, "maximumNewtons", q.maximumNewtons);
        read(p, "minimumSeconds", q.minimumSeconds);
        read(p, "maximumSeconds", q.maximumSeconds);
        read(p, "startSeconds", q.startSeconds);
        read(p, "windowSeconds", q.windowSeconds);
        if (!(q.minimumNewtons >= 0.0f && q.maximumNewtons >= q.minimumNewtons && q.minimumSeconds > 0.0f
                && q.maximumSeconds >= q.minimumSeconds && q.startSeconds >= 0.0f && q.windowSeconds >= 0.0f))
            fail("pushes invalido");
    }
    if (document.contains("body")) {
        const nlohmann::json& b = document["body"];
        LocomotionBodyRange3D& r = task.body;
        // (A faixa [min, max] ou um numero so: o corpo fixo.)
        const auto range = [&](const char* key, float& low, float& high) {
            if (!b.contains(key)) return;
            const nlohmann::json& v = b[key];
            low = v.is_array() ? v.at(0).get<float>() : v.get<float>();
            high = v.is_array() ? v.at(1).get<float>() : low;
        };
        range("heightMeters", r.minimumHeight, r.maximumHeight);
        range("massKg", r.minimumMass, r.maximumMass);
        read(b, "heightStep", r.heightStep);
        read(b, "massStep", r.massStep);
        r.evaluationGrid = b.value("evaluationGrid", r.evaluationGrid);
        if (!(r.minimumHeight >= MinimumBodyHeightMeters && r.maximumHeight <= MaximumBodyHeightMeters
                && r.minimumHeight <= r.maximumHeight && r.minimumMass >= MinimumBodyMassKg
                && r.maximumMass <= MaximumBodyMassKg && r.minimumMass <= r.maximumMass && r.heightStep >= 0.01f
                && r.massStep >= 0.1f))
            fail("body invalido (1,70 a 2,00 m, 60 a 100 kg, passos de ao menos 1 cm e 0,1 kg)");
        // (A faixa so no corpo de hoje e o mesmo que sem o bloco.)
        r.enabled = !(std::lround(r.minimumHeight * 100.0f) == 180 && std::lround(r.maximumHeight * 100.0f) == 180
            && std::lround(r.minimumMass * 10.0f) == 800 && std::lround(r.maximumMass * 10.0f) == 800);
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
        for (const float v : { caps.touchdownMeters, caps.swingUpMeters, caps.swingDownMeters, caps.phaseLeadCycles,
                 caps.recoveryUrgency, caps.cadenceOctaves, caps.strideOctaves, caps.leanRadians, caps.comMeters,
                 caps.pelvisUpMeters, caps.pelvisDownMeters, caps.contactAcceleration, caps.jointRadians, caps.gainOctaves })
            if (!std::isfinite(v) || v < 0.0f) fail("teto invalido em caps (negativo ou nao finito)");
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
        read(r, "getUp", w.getUp);
        read(r, "velocityTail", w.velocityTail);
        read(r, "velocityTailSigma", w.velocityTailSigma);
        read(r, "facingTail", w.facingTail);
        read(r, "facingTailSigma", w.facingTailSigma);
        w.smoothCapUnits = r.value("smoothCapUnits", w.smoothCapUnits);
        read(r, "neckSlackDegrees", w.neckSlackDegrees);
        w.neckRelative = r.value("neckRelative", w.neckRelative);
        w.assistIncludesTorque = r.value("assistIncludesTorque", w.assistIncludesTorque);
        w.contactPerCycle = r.value("contactPerCycle", w.contactPerCycle);
        w.velocitySmoothed = r.value("velocitySmoothed", w.velocitySmoothed);
        w.progress = r.value("progress", w.progress);
        read(r, "progressDelta", w.progressDelta);
        w.forbiddenContact = r.value("forbiddenContact", w.forbiddenContact);
        w.styleWorstPart = r.value("styleWorstPart", w.styleWorstPart);
        if (!(w.progressDelta > 0.0f)) fail("progressDelta invalido");
        if (!(w.velocitySigma > 0.0f) || !(w.facingSigma > 0.0f) || !(w.styleSigma > 0.0f) || !(w.contactSigma > 0.0f)
            || !(w.velocityTailSigma > 0.0f) || !(w.facingTailSigma > 0.0f))
            fail("sigma da nota invalido");
        if (!(w.velocityTail >= 0.0f && w.velocityTail <= 1.0f) || !(w.facingTail >= 0.0f && w.facingTail <= 1.0f))
            fail("a cauda da nota fica entre 0 e 1");
        if (!(w.neckSlackDegrees >= 0.0f)) fail("folga do pescoco invalida");
    }
    if (document.contains("termination")) {
        const nlohmann::json& t = document["termination"];
        read(t, "pelvisHeight", task.fallPelvisHeight);
        read(t, "boundaryMeters", task.boundaryMeters);
        read(t, "groundHalfExtentMeters", task.groundHalfExtentMeters);
        if (!(task.groundHalfExtentMeters > task.boundaryMeters))
            fail("o chao (groundHalfExtentMeters) tem de passar do limite (boundaryMeters)");
    }
    if (document.contains("speeds")) {
        const nlohmann::json& v = document["speeds"];
        read(v, "trot", task.speeds.trot);
        read(v, "sprintForward", task.speeds.sprintForward);
        read(v, "sprintSide", task.speeds.sprintSide);
        read(v, "sprintBackward", task.speeds.sprintBackward);
        if (!(task.speeds.trot > 0.0f && task.speeds.sprintForward > 0.0f && task.speeds.sprintSide > 0.0f
                && task.speeds.sprintBackward > 0.0f))
            fail("velocidades invalidas em speeds");
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
        read(d, "savedStartFraction", drill.savedStartFraction);
        read(d, "pushRate", drill.pushRate);
        read(d, "pushMinimumNewtons", drill.pushMinimumNewtons);
        read(d, "pushMaximumNewtons", drill.pushMaximumNewtons);
        read(d, "holdRate", drill.holdRate);
        read(d, "holdMinimumNewtons", drill.holdMinimumNewtons);
        read(d, "holdMaximumNewtons", drill.holdMaximumNewtons);
        read(d, "holdMinimumSeconds", drill.holdMinimumSeconds);
        read(d, "holdMaximumSeconds", drill.holdMaximumSeconds);
        if (!(drill.holdRate >= 0.0f) || !(drill.holdMaximumNewtons >= drill.holdMinimumNewtons)
            || !(drill.holdMinimumSeconds > 0.0f) || !(drill.holdMaximumSeconds >= drill.holdMinimumSeconds))
            fail("empurrao continuo invalido em " + drill.id);
        drill.getUp = d.value("getUp", false);
        drill.singlePush = d.value("singlePush", false);
        read(d, "settleSeconds", drill.settleSeconds);
        if (drill.singlePush && (drill.getUp || !(drill.settleSeconds > 0.0f)))
            fail("singlePush pede settleSeconds > 0 e nao vale para o levantar (" + drill.id + ")");
        read(d, "knockdownMinimumNewtons", drill.knockdownMinimumNewtons);
        read(d, "knockdownMaximumNewtons", drill.knockdownMaximumNewtons);
        drill.category = d.value("category", std::string());
        if (drill.pushMaximumNewtons >= 0.0f && drill.pushMaximumNewtons < drill.pushMinimumNewtons)
            fail("pushMaximumNewtons menor que o minimo em " + drill.id);
        if (!(drill.pushRate >= 0.0f)) fail("pushRate invalido em " + drill.id);
        if (!(drill.savedStartFraction >= 0.0f && drill.savedStartFraction <= 1.0f))
            fail("savedStartFraction fora de 0..1 em " + drill.id);
        drill.pilot = pilotScriptFromJson3D(d.value("pilot", nlohmann::json::object()), drill.id);
        // (O piloto do arco nao segue a velocidade de um instante guardado.)
        if (drill.savedStartFraction > 0.0f && drill.pilot.kind == "arc")
            fail("savedStartFraction nao vale para o arco (" + drill.id + ")");
        if (d.contains("minimum")) {
            read(d["minimum"], "fallFraction", drill.maximumFallFraction);
            read(d["minimum"], "velocityError", drill.maximumVelocityError);
        }
        for (const LocomotionDrill3D& other : task.drills)
            if (other.id == drill.id) fail("exercicio repetido: " + drill.id);
        task.drills.push_back(std::move(drill));
    }
    if (document.contains("savedStarts")) {
        const nlohmann::json& g = document["savedStarts"];
        task.savedStarts.capacity = g.value("capacity", task.savedStarts.capacity);
        read(g, "surviveSeconds", task.savedStarts.surviveSeconds);
        read(g, "minimumSpeed", task.savedStarts.minimumSpeed);
        if (task.savedStarts.capacity == 0 || !(task.savedStarts.surviveSeconds > 0.0f)) fail("savedStarts invalido");
    }
    if (document.contains("assistSchedule")) {
        const nlohmann::json& a = document["assistSchedule"];
        LocomotionAssistSchedule3D& s = task.assistSchedule;
        s.enabled = a.value("enabled", s.enabled);
        read(a, "zeroFraction", s.zeroFraction);
        read(a, "minimum", s.minimumPercent);
        read(a, "maximum", s.maximumPercent);
        read(a, "changesPerSecond", s.changesPerSecond);
        if (!(s.changesPerSecond >= 0.0f)) fail("assistSchedule.changesPerSecond invalido");
        if (!(s.zeroFraction >= 0.0f && s.zeroFraction <= 1.0f) || !(s.minimumPercent >= 0.0f)
            || !(s.maximumPercent >= s.minimumPercent) || !(s.maximumPercent <= 100.0f))
            fail("assistSchedule invalido");
    }
    // O curriculo de velocidade: os tetos por exercicio e celula (o treinador
    // os abre e reescreve; aqui so le). Exercicio sem tetos: sem corte.
    task.speedCurriculum.caps.assign(task.drills.size(), {});
    if (document.contains("speedCurriculum")) {
        const nlohmann::json& c = document["speedCurriculum"];
        task.speedCurriculum.enabled = c.value("enabled", false);
        read(c, "frontier", task.speedCurriculum.frontier);
        if (!(task.speedCurriculum.frontier > 0.0f)) fail("speedCurriculum.frontier invalido");
        if (c.contains("caps")) {
            for (std::size_t d = 0; d < task.drills.size(); ++d) {
                const auto found = c["caps"].find(task.drills[d].id);
                if (found == c["caps"].end()) continue;
                if (!found->is_array() || found->size() != LocomotionCommandCells3D)
                    fail("speedCurriculum.caps." + task.drills[d].id + " precisa de 16 tetos");
                for (std::size_t k = 0; k < LocomotionCommandCells3D; ++k)
                    task.speedCurriculum.caps[d][k] = (*found)[k].get<float>();
            }
        }
    }
    return task;
}

std::vector<PlayerBody3D> locomotionBodyGrid3D(const LocomotionBodyRange3D& range) {
    if (!range.enabled) return { PlayerBody3D {} };
    // (Inteiros: os passos nao acumulam erro; o ultimo ponto e sempre o canto.)
    const auto points = [](float low, float high, float step, float unit) {
        const long first = std::lround(low * unit), last = std::lround(high * unit);
        const long delta = std::max(1L, std::lround(step * unit));
        std::vector<long> out;
        for (long v = first; v < last; v += delta) out.push_back(v);
        out.push_back(last);
        return out;
    };
    std::vector<PlayerBody3D> grid;
    for (long cm : points(range.minimumHeight, range.maximumHeight, range.heightStep, 100.0f))
        for (long dg : points(range.minimumMass, range.maximumMass, range.massStep, 10.0f))
            grid.push_back(clampedBody3D({ static_cast<float>(cm) / 100.0f, static_cast<float>(dg) / 10.0f }));
    return grid;
}

std::size_t locomotionCommandCell3D(float directionRelativeToCamera, bool cameraTurning) {
    constexpr float Sector = 6.28318530718f / static_cast<float>(LocomotionCommandBins3D);
    const float wrapped = std::remainder(directionRelativeToCamera, 6.28318530718f);
    long bin = std::lround(wrapped / Sector);
    bin = ((bin % static_cast<long>(LocomotionCommandBins3D)) + static_cast<long>(LocomotionCommandBins3D))
        % static_cast<long>(LocomotionCommandBins3D);
    return static_cast<std::size_t>(bin) * 2 + (cameraTurning ? 1 : 0);
}

} // namespace MatterEngine
