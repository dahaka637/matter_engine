#include "Engine/RL/Supervisor/TrainingEvolution3D.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace MatterEngine {
namespace {

constexpr double None = TrainingScoreParts3D::None;
constexpr float NoneFloat = std::numeric_limits<float>::quiet_NaN();

bool finite(double value) { return std::isfinite(value); }

// A soma de um exercicio numa fatia, pelos episodios (cada medida so com os
// episodios das rodadas que a trazem).
struct Sum {
    double episodes = 0.0;
    double falls = 0.0, fallsEpisodes = 0.0;
    double stood = 0.0, stoodEpisodes = 0.0;
    double error = 0.0, errorEpisodes = 0.0;
    double requested = 0.0, requestedEpisodes = 0.0;
    double speed = 0.0, speedEpisodes = 0.0;
    double facing = 0.0, facingEpisodes = 0.0;
    double slip = 0.0, slipEpisodes = 0.0;
    double posture = 0.0, postureEpisodes = 0.0;
    double reward = 0.0, seconds = 0.0;

    void add(const TrainingDrillPoint3D& point) {
        const double n = point.episodes;
        if (!(n > 0.0)) return;
        episodes += n;
        const auto put = [n](double value, double& sum, double& count) {
            if (!finite(value)) return;
            sum += value * n;
            count += n;
        };
        put(point.falls, falls, fallsEpisodes);
        put(point.stood, stood, stoodEpisodes);
        put(point.velocityError, error, errorEpisodes);
        put(point.requestedSpeed, requested, requestedEpisodes);
        put(point.bodySpeed, speed, speedEpisodes);
        put(point.facingError, facing, facingEpisodes);
        put(point.slip, slip, slipEpisodes);
        put(point.postureError, posture, postureEpisodes);
        if (finite(point.episodeReturn) && finite(point.seconds) && point.seconds > 0.0) {
            reward += point.episodeReturn * n;
            seconds += point.seconds * n;
        }
    }

    void add(const Sum& other) {
        episodes += other.episodes;
        falls += other.falls;
        fallsEpisodes += other.fallsEpisodes;
        stood += other.stood;
        stoodEpisodes += other.stoodEpisodes;
        error += other.error;
        errorEpisodes += other.errorEpisodes;
        requested += other.requested;
        requestedEpisodes += other.requestedEpisodes;
        speed += other.speed;
        speedEpisodes += other.speedEpisodes;
        facing += other.facing;
        facingEpisodes += other.facingEpisodes;
        slip += other.slip;
        slipEpisodes += other.slipEpisodes;
        posture += other.posture;
        postureEpisodes += other.postureEpisodes;
        reward += other.reward;
        seconds += other.seconds;
    }

    [[nodiscard]] TrainingDrillMeasure3D measure() const {
        const auto mean = [](double sum, double count) { return count > 0.0 ? sum / count : None; };
        TrainingDrillMeasure3D out;
        out.episodes = episodes;
        out.falls = mean(falls, fallsEpisodes);
        out.stood = mean(stood, stoodEpisodes);
        out.velocityError = mean(error, errorEpisodes);
        out.requestedSpeed = mean(requested, requestedEpisodes);
        out.bodySpeed = mean(speed, speedEpisodes);
        out.facingError = mean(facing, facingEpisodes);
        out.slip = mean(slip, slipEpisodes);
        out.postureError = mean(posture, postureEpisodes);
        out.rewardPerSecond = seconds > 0.0 ? reward / seconds : None;
        return out;
    }
};

float percent(double part) { return finite(part) ? static_cast<float>(part * 100.0) : NoneFloat; }
float value(double v) { return finite(v) ? static_cast<float>(v) : NoneFloat; }

} // namespace

double TrainingScoreParts3D::index() const {
    double sum = 0.0, weight = 0.0;
    const auto take = [&](double part, double partWeight) {
        if (!finite(part)) return;
        sum += part * partWeight;
        weight += partWeight;
    };
    take(standing, StandingWeight);
    take(tracking, TrackingWeight);
    take(stability, StabilityWeight);
    take(reach, ReachWeight);
    return weight > 0.0 ? 100.0 * sum / weight : None;
}

TrainingScoreParts3D trainingScoreParts3D(const TrainingDrillMeasure3D& measure, const TrainingDrillInfo3D& info) {
    TrainingScoreParts3D parts;
    if (!(measure.episodes > 0.0)) return parts;
    if (info.getUp && finite(measure.stood)) parts.standing = std::clamp(measure.stood, 0.0, 1.0);
    else if (finite(measure.falls)) parts.standing = std::clamp(1.0 - measure.falls, 0.0, 1.0);
    if (finite(measure.velocityError)) {
        const double asked = finite(measure.requestedSpeed) ? measure.requestedSpeed : 0.0;
        parts.tracking = std::clamp(1.0 - measure.velocityError / std::max(asked, 1.0), 0.0, 1.0);
    }
    if (info.goalSpeed > 0.0 && finite(measure.bodySpeed))
        parts.reach = std::clamp(measure.bodySpeed / info.goalSpeed, 0.0, 1.0);
    double sum = 0.0;
    int count = 0;
    const auto kernel = [&](double measured, double scale, double& part) {
        if (!finite(measured)) return;
        part = std::exp(-std::max(measured, 0.0) / scale);
        sum += part;
        ++count;
    };
    kernel(measure.facingError, 0.5, parts.facing);
    kernel(measure.slip, 0.5, parts.footing);
    kernel(measure.postureError, 0.15, parts.trunk);
    if (count > 0) parts.stability = sum / count;
    return parts;
}

TrainingCharts3D trainingCharts3D(const TrainingHistory3D& history, const std::vector<TrainingDrillInfo3D>& task,
    const std::string& drill, std::size_t slices) {
    TrainingCharts3D out;
    // As rodadas do periodo (as que trazem exercicios), em ordem, com o horario de cada uma.
    std::vector<std::pair<std::uint64_t, double>> rounds;
    rounds.reserve(history.drills.size());
    for (const TrainingDrillPoint3D& point : history.drills) rounds.push_back({ point.iteration, point.time });
    std::sort(rounds.begin(), rounds.end());
    rounds.erase(std::unique(rounds.begin(), rounds.end(),
                     [](const auto& a, const auto& b) { return a.first == b.first; }),
        rounds.end());
    if (rounds.empty() || slices == 0) return out;
    slices = std::min(slices, rounds.size());
    // Cada fatia com o mesmo numero de rodadas; o ponto dela e a ultima rodada.
    std::unordered_map<std::uint64_t, std::size_t> sliceOf;
    sliceOf.reserve(rounds.size());
    out.times.assign(slices, 0.0);
    out.iterations.assign(slices, 0);
    for (std::size_t i = 0; i < rounds.size(); ++i) {
        const std::size_t slice = i * slices / rounds.size();
        sliceOf[rounds[i].first] = slice;
        out.times[slice] = rounds[i].second;
        out.iterations[slice] = rounds[i].first;
    }

    // Os exercicios da conta: os do treino (com peso) e o escolhido.
    std::vector<TrainingDrillInfo3D> infos;
    std::unordered_map<std::string, std::size_t> slot;
    for (const TrainingDrillInfo3D& info : task)
        if (info.weight > 0.0 && slot.emplace(info.id, infos.size()).second) infos.push_back(info);
    std::size_t chosen = infos.size();
    if (!drill.empty()) {
        const auto found = slot.find(drill);
        if (found != slot.end()) {
            chosen = found->second;
        } else {
            TrainingDrillInfo3D info;
            info.id = drill;
            for (const TrainingDrillInfo3D& candidate : task)
                if (candidate.id == drill) info = candidate;
            info.weight = 0.0;
            chosen = infos.size();
            slot.emplace(drill, chosen);
            infos.push_back(info);
        }
    }
    std::vector<std::vector<Sum>> sums(infos.size(), std::vector<Sum>(slices));
    for (const TrainingDrillPoint3D& point : history.drills) {
        const auto found = slot.find(point.drill);
        if (found != slot.end()) sums[found->second][sliceOf[point.iteration]].add(point);
    }
    // A medida de um exercicio na fatia, com as anteriores ate juntar episodios.
    const auto measured = [&](std::size_t drillSlot, std::size_t slice) {
        Sum total;
        for (std::size_t back = 0; back < TrainingChartMaximumTrail && back <= slice; ++back) {
            total.add(sums[drillSlot][slice - back]);
            if (total.episodes >= TrainingChartMinimumEpisodes) break;
        }
        return total.episodes >= TrainingChartMinimumPlotted ? total.measure() : TrainingDrillMeasure3D {};
    };

    // O treino so vira ponto com pelo menos metade do peso da tarefa medido na fatia (no comeco, so os
    // exercicios que caem rapido juntam episodios, e a media saia puxada por eles).
    double taskWeight = 0.0;
    for (const TrainingDrillInfo3D& info : infos) taskWeight += std::max(info.weight, 0.0);
    for (std::vector<float>* series : { &out.evolution, &out.standing, &out.tracking, &out.stability, &out.reach,
             &out.drillEvolution, &out.drillStanding, &out.drillTest, &out.drillSpeed, &out.drillRequested,
             &out.drillStability, &out.drillFacing, &out.drillFooting, &out.drillTrunk, &out.drillReward,
             &out.decisions })
        series->assign(slices, NoneFloat);
    for (std::size_t s = 0; s < slices; ++s) {
        // O treino: cada parte e o indice pela media dos exercicios com peso.
        struct Mean {
            double sum = 0.0, weight = 0.0;
            void take(double v, double w) {
                if (!finite(v)) return;
                sum += v * w;
                weight += w;
            }
            [[nodiscard]] float get(double scale, double needed) const {
                return weight > 0.0 && weight >= needed ? static_cast<float>(sum / weight * scale) : NoneFloat;
            }
        } index, standing, tracking, stability, reach;
        for (std::size_t k = 0; k < infos.size(); ++k) {
            if (!(infos[k].weight > 0.0)) continue;
            const TrainingScoreParts3D parts = trainingScoreParts3D(measured(k, s), infos[k]);
            index.take(parts.index(), infos[k].weight);
            standing.take(parts.standing, infos[k].weight);
            tracking.take(parts.tracking, infos[k].weight);
            stability.take(parts.stability, infos[k].weight);
            reach.take(parts.reach, infos[k].weight);
        }
        const double needed = TrainingChartMinimumCoverage * taskWeight;
        out.evolution[s] = index.get(1.0, needed);
        out.standing[s] = standing.get(100.0, needed);
        out.tracking[s] = tracking.get(100.0, needed);
        out.stability[s] = stability.get(100.0, needed);
        // (A velocidade alcancada so existe nos que andam: basta um.)
        out.reach[s] = reach.get(100.0, 0.0);
        if (chosen < infos.size()) {
            const TrainingDrillMeasure3D m = measured(chosen, s);
            const TrainingScoreParts3D parts = trainingScoreParts3D(m, infos[chosen]);
            out.drillEvolution[s] = value(parts.index());
            out.drillStanding[s] = percent(parts.standing);
            out.drillSpeed[s] = value(m.bodySpeed);
            out.drillRequested[s] = value(m.requestedSpeed);
            out.drillStability[s] = percent(parts.stability);
            out.drillFacing[s] = percent(parts.facing);
            out.drillFooting[s] = percent(parts.footing);
            out.drillTrunk[s] = percent(parts.trunk);
            out.drillReward[s] = value(m.rewardPerSecond);
        }
    }

    // O teste sem ruido do exercicio (as avaliacoes da rede; o classico fica de fora),
    // valendo da rodada dele ate a seguinte.
    if (chosen < infos.size()) {
        std::vector<const TrainingEvaluationPoint3D*> tests;
        for (const TrainingEvaluationPoint3D& point : history.evaluations)
            if (point.drill == drill && point.solver != "classico") tests.push_back(&point);
        std::sort(tests.begin(), tests.end(), [](const auto* a, const auto* b) { return a->iteration < b->iteration; });
        std::size_t next = 0;
        float current = NoneFloat;
        for (std::size_t s = 0; s < slices; ++s) {
            while (next < tests.size() && tests[next]->iteration <= out.iterations[s]) {
                const TrainingEvaluationPoint3D& test = *tests[next++];
                current = percent(infos[chosen].getUp && finite(test.stood) ? test.stood : 1.0 - test.falls);
            }
            out.drillTest[s] = current;
        }
    }
    // As decisoes por segundo: as rodadas amostradas de cada fatia.
    std::vector<double> decisionSum(slices, 0.0), decisionCount(slices, 0.0);
    for (const TrainingIterationPoint3D& point : history.iterations) {
        const auto at = std::lower_bound(out.iterations.begin(), out.iterations.end(), point.iteration);
        if (at == out.iterations.end() || !(point.sps > 0.0)) continue;
        const std::size_t s = static_cast<std::size_t>(at - out.iterations.begin());
        decisionSum[s] += point.sps;
        decisionCount[s] += 1.0;
    }
    for (std::size_t s = 0; s < slices; ++s)
        if (decisionCount[s] > 0.0) out.decisions[s] = static_cast<float>(decisionSum[s] / decisionCount[s]);
    // (As rodadas vem amostradas por tempo: a fatia sem nenhuma fica com a medida mais perto antes, ou depois.)
    float carried = NoneFloat;
    for (float& decision : out.decisions) {
        if (std::isfinite(decision)) carried = decision;
        else decision = carried;
    }
    for (std::size_t s = slices; s-- > 0;) {
        if (std::isfinite(out.decisions[s])) carried = out.decisions[s];
        else out.decisions[s] = carried;
    }
    return out;
}

} // namespace MatterEngine
