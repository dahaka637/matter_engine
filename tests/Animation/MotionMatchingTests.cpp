// O Motion Matching do motor (src/Engine/Animation/MotionMatching): fontes sinteticas - um corredor reto em velocidades
// diferentes e um que vira -, a consulta do proprio estado reencontra o quadro, a busca acha a velocidade certa, a
// normalizacao soma 1, o continuar ganha no empate, a previsao acelera, freia e vira, e o clipe estende a raiz.
#include "Engine/Animation/MotionMatching/MotionMatchingDatabase3D.hpp"
#include "Engine/Animation/MotionMatching/MotionMatchingTrajectory3D.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

using namespace MatterEngine;

namespace {

int failures = 0;
void require(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::cerr << "FALHOU: " << what << '\n';
    }
}

// Um corredor: a raiz anda a `speed` m/s num rumo que gira a `turnRate` rad/s; os dois pes balancam a frente e atras
// (1,5 passada/s) ao lado da raiz.
class Runner final : public MotionMatchingSource3D {
public:
    Runner(float speed, float turnRate, float duration, bool loops) : m_speed(speed), m_turn(turnRate), m_duration(duration), m_loops(loops) {}
    float durationSeconds() const override { return m_duration; }
    bool loops() const override { return m_loops; }
    void sample(float t, MotionMatchingPose3D& out) const override {
        const float yaw = m_turn * t;
        Vec3 position;
        if (std::abs(m_turn) < 1.0e-6f) position = { m_speed * t, 0.0f, 0.0f };
        else position = { m_speed / m_turn * std::sin(yaw), m_speed / m_turn * (1.0f - std::cos(yaw)), 0.0f };
        out.rootPosition = position;
        out.rootYaw = yaw;
        const float phase = 2.0f * 3.14159265f * 1.5f * t;
        const float swing = 0.35f * std::min(1.0f, m_speed / 4.0f);
        const auto foot = [&](float side, float forward) {
            const float c = std::cos(yaw), s = std::sin(yaw);
            return Vec3 { position.x + c * forward - s * side, position.y + s * forward + c * side, 0.08f };
        };
        out.points = { foot(0.1f, swing * std::sin(phase)), foot(-0.1f, -swing * std::sin(phase)), foot(0.0f, 0.0f) };
        out.points[2].z = 0.95f;
        const float sway = 0.15f * std::sin(phase);
        out.axes = { Vec3 { -std::sin(yaw + sway), std::cos(yaw + sway), 0.0f }, Vec3 { std::cos(yaw + sway), std::sin(yaw + sway), 0.0f } };
    }
    // O estado exato (as derivadas) para a consulta.
    MotionMatchingQueryInput3D query(float t, std::span<const float> offsets) const {
        MotionMatchingPose3D now, ahead;
        sample(t, now);
        const float h = 1.0e-3f;
        sample(t + h, ahead);
        MotionMatchingQueryInput3D q;
        q.rootPosition = now.rootPosition;
        q.rootYaw = now.rootYaw;
        q.rootVelocity = (ahead.rootPosition - now.rootPosition) / h;
        q.rootYawRate = m_turn;
        q.points = now.points;
        q.axes = now.axes;
        for (std::size_t i = 0; i < now.points.size(); ++i) q.pointVelocities.push_back((ahead.points[i] - now.points[i]) / h);
        for (float o : offsets) {
            MotionMatchingPose3D at, after;
            sample(t + o, at);
            sample(t + o + h, after);
            q.trajectory.push_back({ o, at.rootPosition, at.rootYaw, (after.rootPosition - at.rootPosition) / h });
        }
        return q;
    }

private:
    float m_speed, m_turn, m_duration;
    bool m_loops;
};

} // namespace

int main() {
    const MotionMatchingSchema3D schema = MotionMatchingSchema3D::unrealDefault();
    require(schema.featureSize() == 28, "o esquema padrao da Unreal tem 28 valores (16 da trajetoria, 12 dos pes)");
    require(std::abs(schema.pastHorizonSeconds() - 0.4f) < 1e-6f && std::abs(schema.futureHorizonSeconds() - 0.7f) < 1e-6f,
        "os horizontes do padrao sao 0,4 s e 0,7 s");
    // (PSS_Default: 18 da trajetoria - 2+4+4+6+2 - e 11 dos ossos - 3+3+3+2; PSS_Stop: 16+11; PSS_Idle: 16 + 3+3+3 + 3+2+2.)
    require(MotionMatchingSchema3D::gaspDefault().featureSize() == 29 && MotionMatchingSchema3D::gaspStop().featureSize() == 29
            && MotionMatchingSchema3D::gaspIdle().featureSize() == 32,
        "os esquemas do Game Animation Sample (" + std::to_string(MotionMatchingSchema3D::gaspDefault().featureSize()) + ", "
            + std::to_string(MotionMatchingSchema3D::gaspStop().featureSize()) + ", "
            + std::to_string(MotionMatchingSchema3D::gaspIdle().featureSize()) + ")");

    const Runner slow(2.0f, 0.0f, 3.0f, false), mid(4.0f, 0.0f, 3.0f, false), fast(6.0f, 0.0f, 3.0f, false),
        curve(4.0f, 1.2f, 3.0f, false), loop(3.0f, 0.0f, 2.0f, true);
    MotionMatchingDatabaseGroup3D runs { "corridas", schema, { &slow, &mid, &fast, &curve }, 0.0f, -0.005f, -0.01f };
    MotionMatchingDatabaseGroup3D cycles { "ciclo", schema, { &loop }, 0.0f, -0.005f, -0.01f };
    const MotionMatchingDatabase3D db({ runs, cycles });
    const std::size_t perClip = static_cast<std::size_t>(std::floor((3.0f - 0.3f) * 30.0f + 1e-3f)) + 1;
    require(db.size() == 4 * perClip + 60 && db.groupEnd(0) == 4 * perClip,
        "os quadros: 30/s sem os ultimos 0,3 s; o ciclo de 2 s com 60");

    // A normalizacao comum: com os desvios de todos os grupos, os pesos ao quadrado somam 1 em cada grupo.
    {
        double total = 0.0;
        for (const MotionMatchingSchema3D::Leaf& leaf : schema.leaves()) {
            const std::size_t n = db.size();
            std::vector<double> mean(leaf.size, 0.0);
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t d = 0; d < leaf.size; ++d) mean[d] += db.features(i)[leaf.begin + d] / n;
            double dev = 0.0;
            for (std::size_t i = 0; i < n; ++i) {
                double sum = 0.0;
                for (std::size_t d = 0; d < leaf.size; ++d) sum += std::pow(db.features(i)[leaf.begin + d] - mean[d], 2);
                dev += std::sqrt(sum) / n;
            }
            const double w = db.weightsSqrt(1)[leaf.begin] * (dev <= 1e-3 ? 1.0 : dev);
            total += w * w * leaf.size;
        }
        require(std::abs(total - 1.0) < 1e-3, "os pesos normalizados somam 1 (soma " + std::to_string(total) + ")");
        require(db.weightsSqrt(0)[0] == db.weightsSqrt(1)[0], "o mesmo esquema tem os mesmos pesos nos dois grupos (desvio comum)");
    }

    // A consulta do proprio estado (derivadas exatas) reencontra o quadro do banco.
    std::vector<float> offsets;
    for (const MotionMatchingTrajectorySample3D& s : schema.trajectory) offsets.push_back(s.offsetSeconds);
    std::vector<std::vector<float>> queries(2, std::vector<float>(db.featureSize(0)));
    std::vector<float> stored(db.featureSize(0));
    const auto ask = [&](const Runner& r, float t) {
        for (std::size_t g = 0; g < 2; ++g) db.queryFeatures(g, r.query(t, offsets), queries[g]);
    };
    for (const auto& [source, index, t] : { std::tuple { &mid, std::size_t { 1 }, 1.5f }, std::tuple { &curve, std::size_t { 3 }, 1.2f } }) {
        ask(*source, t);
        db.sourceFeatures(0, index, t, stored);
        float worst = 0.0f;
        for (std::size_t d = 0; d < stored.size(); ++d) worst = std::max(worst, std::abs(queries[0][d] - stored[d]) * db.weightsSqrt(0)[d]);
        require(worst < 0.05f, "a consulta do estado e o quadro do banco batem (pior diferenca ponderada " + std::to_string(worst) + ")");
        // (Os corredores sinteticos repetem os pes a cada passada: varios quadros iguais - vale o custo do proprio quadro.)
        const MotionMatchingResult3D found = db.search(queries);
        const float own = db.cost(db.entryAt(0, index, t), queries[0]);
        require(found.valid() && db.entries()[found.index].source == index && found.cost <= own + 1e-6f && own < 0.01f,
            "a busca acha o proprio quadro (fonte " + std::to_string(found.valid() ? db.entries()[found.index].source : 99)
                + ", custo " + std::to_string(own) + ")");
    }

    // A busca pela velocidade: 5,5 m/s reto acha o de 6, 2,3 acha o de 2; so o grupo do ciclo, o ciclo.
    for (const auto& [speed, expected] : { std::pair { 5.5f, std::size_t { 2 } }, std::pair { 2.3f, std::size_t { 0 } } }) {
        ask(Runner(speed, 0.0f, 3.0f, false), 1.0f);
        const MotionMatchingResult3D found = db.search(queries);
        require(found.valid() && db.entries()[found.index].group == 0 && db.entries()[found.index].source == expected,
            "a " + std::to_string(speed) + " m/s a busca escolhe a fonte " + std::to_string(expected));
        MotionMatchingSearchOptions3D only;
        only.groups = 2u;
        const MotionMatchingResult3D cycled = db.search(queries, only);
        require(cycled.valid() && db.entries()[cycled.index].group == 1, "a busca so no grupo pedido");
    }

    // O continuar ganha no empate (o viés de -0,01) e o excluido nao volta.
    {
        db.sourceFeatures(0, 1, 1.0f, queries[0]);
        const std::size_t here = db.entryAt(0, 1, 1.0f);
        MotionMatchingSearchOptions3D keep;
        keep.groups = 1u;
        keep.continuingIndex = here;
        const MotionMatchingResult3D kept = db.search(queries, keep);
        require(kept.continuing && kept.index == here, "o quadro que ja toca continua no empate");
        const std::vector<std::size_t> excluded { here };
        MotionMatchingSearchOptions3D skip;
        skip.groups = 1u;
        skip.excluded = excluded;
        const MotionMatchingResult3D other = db.search(queries, skip);
        require(other.valid() && other.index != here, "o quadro excluido nao e escolhido");
        const std::vector<MotionMatchingResult3D> near = db.nearest(queries, 5, 1u);
        require(near.size() == 5 && near.front().cost <= db.cost(here, queries[0]) + 1e-6f && near[0].cost <= near[4].cost,
            "os k mais perto em ordem");
    }

    // O ciclo continua a raiz e a ponta do clipe a estende pela velocidade (a posicao da amostra de 0,7 s e o valor 12).
    {
        db.sourceFeatures(1, 0, 1.9f, stored);
        require(std::abs(stored[12] - 2.1f) < 0.02f, "o ciclo estende a raiz (" + std::to_string(stored[12]) + " m)");
        db.sourceFeatures(0, 1, 2.7f, stored);
        require(std::abs(stored[12] - 2.8f) < 0.02f, "a ponta do clipe estende a raiz pela velocidade (" + std::to_string(stored[12]) + " m)");
    }

    // O esquema do Game Animation Sample: a consulta do estado bate com o banco (a direcao da velocidade, o pe em relacao ao
    // outro, o eixo da pelve) e a busca acha a velocidade.
    {
        const MotionMatchingDatabase3D gasp({ { "padrao", MotionMatchingSchema3D::gaspDefault(), { &slow, &mid, &fast, &curve } },
            { "parado", MotionMatchingSchema3D::gaspIdle(), { &loop } } });
        std::vector<float> times;
        for (const auto& s : MotionMatchingSchema3D::gaspDefault().trajectory) times.push_back(s.offsetSeconds);
        std::vector<std::vector<float>> q { std::vector<float>(gasp.featureSize(0)), std::vector<float>(gasp.featureSize(1)) };
        std::vector<float> own(gasp.featureSize(0));
        for (std::size_t g = 0; g < 2; ++g) gasp.queryFeatures(g, curve.query(1.2f, times), q[g]);
        gasp.sourceFeatures(0, 3, 1.2f, own);
        float worst = 0.0f;
        for (std::size_t d = 0; d < own.size(); ++d) worst = std::max(worst, std::abs(q[0][d] - own[d]) * gasp.weightsSqrt(0)[d]);
        require(worst < 0.05f, "o esquema do GASP: a consulta bate com o banco (" + std::to_string(worst) + ")");
        for (std::size_t g = 0; g < 2; ++g) gasp.queryFeatures(g, Runner(5.6f, 0.0f, 3.0f, false).query(1.0f, times), q[g]);
        const MotionMatchingResult3D found = gasp.search(q);
        require(found.valid() && gasp.entries()[found.index].group == 0 && gasp.entries()[found.index].source == 2,
            "o esquema do GASP acha a velocidade");
    }

    // A previsao: do parado a 4 m/s para a frente, a velocidade sobe ate 4 e para; sem pedido, freia ate parar; o rumo vai
    // ao desejado (e, com a taxa negativa, na hora).
    {
        MotionMatchingMovementSettings3D settings;
        MotionMatchingMovementState3D state;
        state.desiredDirection = { 1.0f, 0.0f, 0.0f };
        state.desiredSpeed = 4.0f;
        const std::vector<float> times { 0.25f, 0.5f, 1.0f, 2.0f };
        const auto go = predictMotionTrajectory3D(settings, state, times);
        require(go.size() == 4 && go[0].velocity.x > 0.5f && go[3].velocity.x <= 4.0f + 1e-4f && go[3].velocity.x > 3.9f
                && go[1].position.x > go[0].position.x, "a previsao acelera ate o pedido");
        state.velocity = { 4.0f, 0.0f, 0.0f };
        state.desiredSpeed = 0.0f;
        const auto stop = predictMotionTrajectory3D(settings, state, times);
        require(stop.back().velocity.length() < 1e-3f && stop.back().position.x > 0.0f && stop.back().position.x < 4.0f,
            "sem pedido a previsao freia e para");
        settings.rotationRadiansPerSecond = 2.0f;
        state.hasDesiredFacing = true;
        state.desiredFacingYaw = 1.5f;
        const auto face = predictMotionTrajectory3D(settings, state, times);
        require(std::abs(face.back().yaw - 1.5f) < 1e-3f && face.front().yaw > 0.0f && face.front().yaw < 1.5f, "o rumo vai ao desejado");
        settings.rotationRadiansPerSecond = -1.0f;
        const auto instant = predictMotionTrajectory3D(settings, state, times);
        require(std::abs(instant.front().yaw - 1.5f) < 1e-4f, "com a taxa negativa o rumo vai na hora");
    }

    // O historico interpola o passado.
    {
        MotionMatchingTrajectoryHistory3D history(1.0f);
        for (int i = 0; i <= 30; ++i) history.record(i / 30.0f, { i / 30.0f * 2.0f, 0.0f, 0.0f }, 0.0f, { 2.0f, 0.0f, 0.0f });
        MotionMatchingTrajectoryPoint3D p;
        require(history.sample(-0.4f, p) && std::abs(p.position.x - 1.2f) < 1e-3f, "o historico de 0,4 s atras");
    }

    if (failures == 0) std::cout << "MotionMatching: todos os testes passaram\n";
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
