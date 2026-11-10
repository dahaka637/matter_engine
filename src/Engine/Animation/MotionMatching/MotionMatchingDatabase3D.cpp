#include "Engine/Animation/MotionMatching/MotionMatchingDatabase3D.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <thread>

namespace MatterEngine {

namespace {

// Um vetor do mundo no referencial de um rumo (girado por -yaw em torno de Z).
Vec3 toLocal(float yaw, Vec3 v) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * v.x + s * v.y, -s * v.x + c * v.y, v.z };
}
Vec3 rotateZ(float yaw, Vec3 v) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}
// A pose levada pela transformacao que leva a raiz `from` (posicao, rumo) para `to`.
void carry(MotionMatchingPose3D& pose, Vec3 fromPosition, float fromYaw, Vec3 toPosition, float toYaw) {
    const float turn = toYaw - fromYaw;
    pose.rootPosition = toPosition + rotateZ(turn, pose.rootPosition - fromPosition);
    pose.rootYaw += turn;
    for (Vec3& p : pose.points) p = toPosition + rotateZ(turn, p - fromPosition);
    for (Vec3& a : pose.axes) a = rotateZ(turn, a);
}
// A direcao da velocidade (o bNormalize do PoseSearch: o vetor em cm/s preso no comprimento 1 - unitario acima de 1 cm/s).
Vec3 direction(Vec3 v) {
    const Vec3 centimeters = v * 100.0f;
    const float length = centimeters.length();
    return length > 1.0f ? centimeters / length : centimeters;
}
// O eixo no referencial da raiz, sem o Z e renormalizado (o Heading com StripZ).
std::pair<float, float> heading(float yaw, Vec3 axis) {
    const Vec3 local = toLocal(yaw, axis);
    const float length = std::hypot(local.x, local.y);
    return length > 1.0e-6f ? std::pair { local.x / length, local.y / length } : std::pair { 1.0f, 0.0f };
}

using Feature = MotionMatchingBoneFeature3D;

// [begin, end) dividido entre os nucleos (as fontes sao so lidas: amostrar e const).
template <typename Function> void parallelFor(std::size_t begin, std::size_t end, Function&& function) {
    const std::size_t count = end > begin ? end - begin : 0;
    const std::size_t workers = std::min<std::size_t>(std::max(1u, std::min(8u, std::thread::hardware_concurrency())), std::max<std::size_t>(count / 64, 1));
    if (workers <= 1) {
        for (std::size_t i = begin; i < end; ++i) function(i);
        return;
    }
    std::vector<std::thread> threads;
    for (std::size_t k = 0; k < workers; ++k)
        threads.emplace_back([&, k] {
            for (std::size_t i = begin + k; i < end; i += workers) function(i);
        });
    for (std::thread& t : threads) t.join();
}

} // namespace

MotionMatchingDatabase3D::MotionMatchingDatabase3D(std::vector<MotionMatchingDatabaseGroup3D> groups) {
    if (groups.size() > 64) throw std::invalid_argument("MotionMatchingDatabase3D: no maximo 64 grupos");
    for (MotionMatchingDatabaseGroup3D& spec : groups) {
        if (spec.schema.sampleRate <= 0.0f || spec.schema.finiteDeltaSeconds <= 0.0f)
            throw std::invalid_argument("MotionMatchingDatabase3D: sampleRate e finiteDeltaSeconds precisam ser positivos");
        for (const MotionMatchingSource3D* source : spec.sources)
            if (source == nullptr) throw std::invalid_argument("MotionMatchingDatabase3D: fonte nula no grupo " + spec.name);
        Group g;
        g.spec = std::move(spec);
        g.leaves = g.spec.schema.leaves();
        g.size = g.spec.schema.featureSize();
        m_groups.push_back(std::move(g));
    }
    // Os quadros: i / sampleRate, de 0 ao fim (no ciclo, sem o quadro que repete o primeiro; fora dele, sem o fim excluido).
    for (std::size_t gi = 0; gi < m_groups.size(); ++gi) {
        Group& g = m_groups[gi];
        g.first = m_entries.size();
        for (std::size_t s = 0; s < g.spec.sources.size(); ++s) {
            g.firstEntry.push_back(m_entries.size());
            const MotionMatchingSource3D& source = *g.spec.sources[s];
            const float duration = std::max(0.0f, source.durationSeconds());
            const float end = source.loops() ? duration : duration - g.spec.schema.excludeEndSeconds;
            const float addend = g.spec.baseCostBias + (source.loops() ? g.spec.loopingCostBias : 0.0f);
            for (int i = 0;; ++i) {
                const float t = static_cast<float>(i) / g.spec.schema.sampleRate;
                if (source.loops() ? t >= end - 1.0e-5f : t > end + 1.0e-5f) break;
                m_entries.push_back({ gi, s, t, addend });
            }
        }
        g.firstEntry.push_back(m_entries.size());
        g.last = m_entries.size();
        g.values.assign((g.last - g.first) * g.size, 0.0f);
        parallelFor(g.first, g.last, [&](std::size_t i) {
            sourceFeatures(gi, m_entries[i].source, m_entries[i].timeSeconds,
                std::span<float>(g.values).subspan((i - g.first) * g.size, g.size));
        });
    }
    // O desvio de cada rotulo, em todos os grupos (o NormalizeWithCommonSchema).
    struct Stat {
        std::size_t size = 0;
        std::vector<double> mean;
        double deviation = 0.0;
        std::size_t rows = 0;
    };
    std::map<std::string, Stat> stats;
    for (const Group& g : m_groups)
        for (const MotionMatchingSchema3D::Leaf& leaf : g.leaves) {
            Stat& st = stats[leaf.label];
            if (st.size == 0) {
                st.size = leaf.size;
                st.mean.assign(leaf.size, 0.0);
            } else if (st.size != leaf.size) {
                throw std::invalid_argument("MotionMatchingDatabase3D: o rotulo " + leaf.label + " com tamanhos diferentes");
            }
            const std::size_t n = g.last - g.first;
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t d = 0; d < leaf.size; ++d) st.mean[d] += g.values[i * g.size + leaf.begin + d];
            st.rows += n;
        }
    for (auto& [label, st] : stats)
        for (double& m : st.mean) m /= std::max<std::size_t>(st.rows, 1);
    for (const Group& g : m_groups)
        for (const MotionMatchingSchema3D::Leaf& leaf : g.leaves) {
            Stat& st = stats[leaf.label];
            const std::size_t n = g.last - g.first;
            for (std::size_t i = 0; i < n; ++i) {
                double sum = 0.0;
                for (std::size_t d = 0; d < leaf.size; ++d) {
                    const double x = g.values[i * g.size + leaf.begin + d] - st.mean[d];
                    sum += x * x;
                }
                st.deviation += std::sqrt(sum);
            }
        }
    for (Group& g : m_groups) {
        float total = 0.0f;
        for (const MotionMatchingSchema3D::Leaf& leaf : g.leaves) total += std::max(0.0f, leaf.weight) * static_cast<float>(leaf.size);
        g.weightsSqrt.assign(g.size, 0.0f);
        for (const MotionMatchingSchema3D::Leaf& leaf : g.leaves) {
            const Stat& st = stats[leaf.label];
            const double deviation = st.deviation / std::max<std::size_t>(st.rows, 1);
            // (No PoseSearch o piso e 0,1 cm; aqui em metros, 1 mm.)
            const float d = deviation <= 1.0e-3 ? 1.0f : static_cast<float>(deviation);
            const float w = total > 0.0f ? std::sqrt(std::max(0.0f, leaf.weight) / total) / d : 0.0f;
            for (std::size_t k = 0; k < leaf.size; ++k) g.weightsSqrt[leaf.begin + k] = w;
        }
    }
}

std::span<const float> MotionMatchingDatabase3D::features(std::size_t index) const {
    const Group& g = m_groups[m_entries[index].group];
    return std::span<const float>(g.values).subspan((index - g.first) * g.size, g.size);
}

void MotionMatchingDatabase3D::weighted(std::size_t gi, std::span<const float> raw, std::span<float> out) const {
    const Group& g = m_groups[gi];
    for (std::size_t d = 0; d < g.size; ++d) out[d] = raw[d] * g.weightsSqrt[d];
}

void MotionMatchingDatabase3D::sampleExtended(const Group& g, std::size_t s, float seconds, MotionMatchingPose3D& out) const {
    const MotionMatchingSource3D& source = *g.spec.sources[s];
    const float duration = std::max(0.0f, source.durationSeconds());
    if (duration <= 0.0f) {
        source.sample(0.0f, out);
        return;
    }
    if (source.loops()) {
        const float cycles = std::floor(seconds / duration);
        source.sample(seconds - cycles * duration, out);
        if (cycles == 0.0f) return;
        MotionMatchingPose3D start, end;
        source.sample(0.0f, start);
        source.sample(duration, end);
        for (int c = 0; c < static_cast<int>(std::abs(cycles)); ++c) {
            if (cycles > 0.0f) carry(out, start.rootPosition, start.rootYaw, end.rootPosition, end.rootYaw);
            else carry(out, end.rootPosition, end.rootYaw, start.rootPosition, start.rootYaw);
        }
        return;
    }
    // (Fora do clipe que nao repete: a pose da ponta, levada pela velocidade da raiz no ultimo 1/30 s - PoseSearchAssetSampler.)
    if (seconds >= 0.0f && seconds <= duration) {
        source.sample(seconds, out);
        return;
    }
    const float span = std::min(1.0f / 30.0f, duration);
    const bool before = seconds < 0.0f;
    MotionMatchingPose3D inner;
    source.sample(before ? 0.0f : duration, out);
    source.sample(before ? span : duration - span, inner);
    const Vec3 velocity = (before ? inner.rootPosition - out.rootPosition : out.rootPosition - inner.rootPosition) / span;
    const float yawRate = (before ? inner.rootYaw - out.rootYaw : out.rootYaw - inner.rootYaw) / span;
    const float beyond = before ? seconds : seconds - duration;
    carry(out, out.rootPosition, out.rootYaw, out.rootPosition + velocity * beyond, out.rootYaw + yawRate * beyond);
}

void MotionMatchingDatabase3D::sourceFeatures(std::size_t gi, std::size_t s, float seconds, std::span<float> out) const {
    const Group& g = m_groups[gi];
    const MotionMatchingSchema3D& schema = g.spec.schema;
    const float delta = schema.finiteDeltaSeconds;
    MotionMatchingPose3D now, previous, at, behind;
    sampleExtended(g, s, seconds, now);
    sampleExtended(g, s, seconds - delta, previous);
    std::size_t k = 0;
    const auto put2 = [&](float a, float b) {
        out[k++] = a;
        out[k++] = b;
    };
    for (const MotionMatchingTrajectorySample3D& sample : schema.trajectory) {
        sampleExtended(g, s, seconds + sample.offsetSeconds, at);
        Vec3 velocity;
        if (sample.velocity || sample.velocityDirection) {
            sampleExtended(g, s, seconds + sample.offsetSeconds - delta, behind);
            velocity = toLocal(now.rootYaw, (at.rootPosition - behind.rootPosition) / delta);
            velocity.z = 0.0f;
        }
        if (sample.velocity) put2(velocity.x, velocity.y);
        if (sample.velocityDirection) {
            const Vec3 d = direction(velocity);
            put2(d.x, d.y);
        }
        if (sample.position) {
            const Vec3 p = toLocal(now.rootYaw, at.rootPosition - now.rootPosition);
            put2(p.x, p.y);
        }
        if (sample.facing) put2(std::cos(at.rootYaw - now.rootYaw), std::sin(at.rootYaw - now.rootYaw));
    }
    for (const MotionMatchingBoneChannel3D& b : schema.bones) {
        switch (b.feature) {
        case Feature::Position: {
            const Vec3 origin = b.origin == MotionMatchingBoneChannel3D::Root ? now.rootPosition : now.points.at(b.origin);
            const Vec3 p = toLocal(now.rootYaw, now.points.at(b.index) - origin);
            out[k++] = p.x;
            out[k++] = p.y;
            out[k++] = p.z;
            break;
        }
        case Feature::Velocity: {
            const Vec3 here = toLocal(now.rootYaw, now.points.at(b.index) - now.rootPosition);
            const Vec3 before = toLocal(previous.rootYaw, previous.points.at(b.index) - previous.rootPosition);
            const Vec3 v = (here - before) / delta;
            out[k++] = v.x;
            out[k++] = v.y;
            out[k++] = v.z;
            break;
        }
        case Feature::Heading: {
            const auto [x, y] = heading(now.rootYaw, now.axes.at(b.index));
            put2(x, y);
            break;
        }
        }
    }
}

void MotionMatchingDatabase3D::queryFeatures(std::size_t gi, const MotionMatchingQueryInput3D& input, std::span<float> out) const {
    const MotionMatchingSchema3D& schema = m_groups[gi].spec.schema;
    // A trajetoria num instante: interpolada entre os pontos dados, presa nas pontas; o instante 0 e o personagem agora.
    const auto at = [&](float offset) {
        MotionMatchingTrajectoryPoint3D p { 0.0f, input.rootPosition, input.rootYaw, input.rootVelocity };
        if (offset == 0.0f || input.trajectory.empty()) return p;
        const auto& path = input.trajectory;
        if (offset <= path.front().offsetSeconds) return path.front();
        if (offset >= path.back().offsetSeconds) return path.back();
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (offset > path[i].offsetSeconds) continue;
            const MotionMatchingTrajectoryPoint3D& a = path[i - 1];
            const MotionMatchingTrajectoryPoint3D& b = path[i];
            const float span = b.offsetSeconds - a.offsetSeconds;
            const float u = span > 1.0e-6f ? (offset - a.offsetSeconds) / span : 1.0f;
            p.offsetSeconds = offset;
            p.position = a.position + (b.position - a.position) * u;
            p.velocity = a.velocity + (b.velocity - a.velocity) * u;
            p.yaw = a.yaw + std::remainder(b.yaw - a.yaw, 6.283185307f) * u;
            return p;
        }
        return path.back();
    };
    std::size_t k = 0;
    const auto put2 = [&](float a, float b) {
        out[k++] = a;
        out[k++] = b;
    };
    for (const MotionMatchingTrajectorySample3D& sample : schema.trajectory) {
        const MotionMatchingTrajectoryPoint3D p = at(sample.offsetSeconds);
        Vec3 velocity = toLocal(input.rootYaw, p.velocity);
        velocity.z = 0.0f;
        if (sample.velocity) put2(velocity.x, velocity.y);
        if (sample.velocityDirection) {
            const Vec3 d = direction(velocity);
            put2(d.x, d.y);
        }
        if (sample.position) {
            const Vec3 local = toLocal(input.rootYaw, p.position - input.rootPosition);
            put2(local.x, local.y);
        }
        if (sample.facing) put2(std::cos(p.yaw - input.rootYaw), std::sin(p.yaw - input.rootYaw));
    }
    for (const MotionMatchingBoneChannel3D& b : schema.bones) {
        switch (b.feature) {
        case Feature::Position: {
            const Vec3 origin = b.origin == MotionMatchingBoneChannel3D::Root ? input.rootPosition : input.points.at(b.origin);
            const Vec3 p = toLocal(input.rootYaw, input.points.at(b.index) - origin);
            out[k++] = p.x;
            out[k++] = p.y;
            out[k++] = p.z;
            break;
        }
        case Feature::Velocity: {
            // (A derivada da posicao no referencial que gira com a raiz - a mesma diferenca finita do banco, no limite.)
            const Vec3 local = toLocal(input.rootYaw, input.points.at(b.index) - input.rootPosition);
            const Vec3 relative = toLocal(input.rootYaw, input.pointVelocities.at(b.index) - input.rootVelocity);
            out[k++] = relative.x + input.rootYawRate * local.y;
            out[k++] = relative.y - input.rootYawRate * local.x;
            out[k++] = relative.z;
            break;
        }
        case Feature::Heading: {
            const auto [x, y] = heading(input.rootYaw, input.axes.at(b.index));
            put2(x, y);
            break;
        }
        }
    }
}

float MotionMatchingDatabase3D::cost(std::size_t index, std::span<const float> query) const {
    const MotionMatchingEntry3D& e = m_entries[index];
    const Group& g = m_groups[e.group];
    const float* x = g.values.data() + (index - g.first) * g.size;
    float sum = 0.0f;
    for (std::size_t d = 0; d < g.size; ++d) {
        const float diff = (query[d] - x[d]) * g.weightsSqrt[d];
        sum += diff * diff;
    }
    return sum + e.costAddend;
}

MotionMatchingResult3D MotionMatchingDatabase3D::search(std::span<const std::vector<float>> queries,
    const MotionMatchingSearchOptions3D& options) const {
    MotionMatchingResult3D best;
    // (O continuar primeiro, com o viés do grupo dele; os outros so ganham com custo estritamente menor - PoseSearchLibrary.)
    if (options.continuingIndex < m_entries.size()) {
        const std::size_t g = m_entries[options.continuingIndex].group;
        best = { options.continuingIndex, cost(options.continuingIndex, queries[g]) + m_groups[g].spec.continuingCostBias, true };
    }
    for (std::size_t g = 0; g < m_groups.size(); ++g) {
        if (((options.groups >> g) & 1u) == 0) continue;
        for (std::size_t i = m_groups[g].first; i < m_groups[g].last; ++i) {
            if (i == options.continuingIndex) continue;
            if (std::find(options.excluded.begin(), options.excluded.end(), i) != options.excluded.end()) continue;
            const float c = cost(i, queries[g]);
            if (c < best.cost) best = { i, c, false };
        }
    }
    return best;
}

std::vector<MotionMatchingResult3D> MotionMatchingDatabase3D::nearest(std::span<const std::vector<float>> queries, std::size_t k,
    std::uint64_t groups) const {
    std::vector<MotionMatchingResult3D> all;
    for (std::size_t g = 0; g < m_groups.size(); ++g) {
        if (((groups >> g) & 1u) == 0) continue;
        for (std::size_t i = m_groups[g].first; i < m_groups[g].last; ++i) all.push_back({ i, cost(i, queries[g]), false });
    }
    k = std::min(k, all.size());
    std::partial_sort(all.begin(), all.begin() + static_cast<std::ptrdiff_t>(k), all.end(),
        [](const MotionMatchingResult3D& a, const MotionMatchingResult3D& b) { return a.cost < b.cost; });
    all.resize(k);
    return all;
}

std::size_t MotionMatchingDatabase3D::entryAt(std::size_t gi, std::size_t source, float seconds) const {
    const Group& g = m_groups[gi];
    if (source + 1 >= g.firstEntry.size()) return MotionMatchingResult3D::None;
    const std::size_t first = g.firstEntry[source], count = g.firstEntry[source + 1] - first;
    if (count == 0) return MotionMatchingResult3D::None;
    long long i = std::llround(seconds * g.spec.schema.sampleRate);
    if (g.spec.sources[source]->loops())
        i = ((i % static_cast<long long>(count)) + static_cast<long long>(count)) % static_cast<long long>(count);
    if (i < 0 || i >= static_cast<long long>(count)) return MotionMatchingResult3D::None;
    return first + static_cast<std::size_t>(i);
}

} // namespace MatterEngine
