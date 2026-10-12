// A ponte Python do treino de RL (matter_rl_native): o lote de mundos e a
// politica exportada. E a unica parte do projeto que inclui Python. Os
// buffers do lote saem como arrays numpy sem copia (somente leitura; valem
// ate o proximo step), e step() solta o GIL enquanto a fisica roda.
// O Python.h (pelo nanobind) vem antes de qualquer cabecalho do sistema.
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "Engine/RL/Environment/TrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;
using namespace MatterEngine;

namespace {

using FloatView2 = nb::ndarray<nb::numpy, const float, nb::ndim<2>>;
using FloatView1 = nb::ndarray<nb::numpy, const float, nb::ndim<1>>;
using ByteView1 = nb::ndarray<nb::numpy, const std::uint8_t, nb::ndim<1>>;

std::string hex(std::uint64_t value) {
    char text[17];
    std::snprintf(text, sizeof text, "%016llx", static_cast<unsigned long long>(value));
    return text;
}

std::vector<std::string> names(const RLVectorLayout3D& layout) {
    std::vector<std::string> result;
    for (const RLChannel3D& channel : layout.channels) result.push_back(channel.name);
    return result;
}

FloatView2 view2(TrainingBatch3D& batch, std::span<const float> data, std::size_t columns) {
    const std::size_t rows = batch.environmentCount();
    return FloatView2(data.data(), { rows, columns }, nb::find(&batch));
}

// As linhas do dump do MatterModelInspector quadros para um mundo agora: o
// colisor de cada link (C capsula, B caixa) e os contatos (P).
std::string frameLines(const TrainingBatch3D& batch, std::size_t environment) {
    const TrainingEnvironmentView3D view = batch.view(environment);
    std::ostringstream out;
    if (!view.state) return {};
    const RagdollProfile3D& profile = batch.body().profile;
    for (std::size_t i = 0; i < profile.links.size() && i < view.state->links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        const PhysicsBodyState3D& state = view.state->links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    for (const RagdollContactPoint3D& contact : view.state->contacts)
        out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
            << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z << ' '
            << contact.normalImpulseNewtonSeconds << '\n';
    return out.str();
}

} // namespace

NB_MODULE(matter_rl_native, m) {
    m.doc() = "Mundos de treino de RL do Matter Engine (C++, sem janela)";

    nb::class_<TrainingBatch3D>(m, "Batch")
        .def(
            "__init__",
            [](TrainingBatch3D* self, const std::string& task, std::uint32_t environments, std::uint32_t perScene,
                std::uint32_t threads, std::uint64_t seed, bool evaluation, float evaluationSeconds,
                int solverPosition, int solverVelocity, const std::string& posture) {
                TrainingBatchSettings3D settings;
                settings.taskPath = task;
                settings.environmentCount = environments;
                settings.environmentsPerScene = perScene;
                settings.workerThreads = threads;
                settings.seed = seed;
                settings.evaluation = evaluation;
                settings.evaluationEpisodeSeconds = evaluationSeconds;
                settings.solverPositionIterations = solverPosition;
                settings.solverVelocityIterations = solverVelocity;
                settings.posture = posture;
                nb::gil_scoped_release release;
                new (self) TrainingBatch3D(settings);
            },
            "task"_a, "environments"_a = 256, "per_scene"_a = 4, "threads"_a = 0, "seed"_a = 1,
            "evaluation"_a = false, "evaluation_seconds"_a = 60.0f, "solver_position"_a = -1,
            "solver_velocity"_a = -1, "posture"_a = "")
        .def_prop_ro("num_envs", &TrainingBatch3D::environmentCount)
        .def_prop_ro("obs_size", &TrainingBatch3D::observationSize)
        .def_prop_ro("critic_size", &TrainingBatch3D::criticObservationSize)
        .def_prop_ro("action_size", &TrainingBatch3D::actionSize)
        .def_prop_ro("reward_count", &TrainingBatch3D::rewardComponentCount)
        .def_prop_ro("threads", &TrainingBatch3D::workerThreads)
        .def_prop_ro("policy_dt", &TrainingBatch3D::policyStepSeconds)
        .def_prop_ro("body_id", [](const TrainingBatch3D& b) { return b.body().id; })
        .def_prop_ro("body_hash", [](const TrainingBatch3D& b) { return hex(b.body().hash); })
        .def_prop_ro("task_id", [](const TrainingBatch3D& b) { return b.task().id; })
        .def_prop_ro("task_name", [](const TrainingBatch3D& b) { return b.task().displayName; })
        .def_prop_ro("task_hash", [](const TrainingBatch3D& b) { return hex(b.task().hash); })
        .def_prop_ro("profile_path", [](const TrainingBatch3D& b) { return b.body().profilePath.string(); })
        .def_prop_ro("link_ids",
            [](const TrainingBatch3D& b) {
                std::vector<std::string> ids;
                for (const RagdollLinkDefinition3D& link : b.body().profile.links) ids.push_back(link.id);
                return ids;
            })
        .def("task_postures", &TrainingBatch3D::taskPostureIds)
        .def("posture_ids",
            [](const TrainingBatch3D& b) {
                std::vector<std::string> ids;
                for (const RLPosture3D& posture : b.body().postures) ids.push_back(posture.id);
                return ids;
            })
        .def("set_posture", &TrainingBatch3D::setPosture, "posture"_a)
        .def("observation_names", [](const TrainingBatch3D& b) { return names(b.observationLayout()); })
        .def("critic_names", [](const TrainingBatch3D& b) { return names(b.criticObservationLayout()); })
        .def("action_names", [](const TrainingBatch3D& b) { return names(b.actionLayout()); })
        .def("reward_names",
            [](const TrainingBatch3D&) {
                return std::vector<std::string>(TrainingRewardComponentNames3D.begin(), TrainingRewardComponentNames3D.end());
            })
        .def("layout_ids",
            [](const TrainingBatch3D& b) {
                return std::vector<std::string> { b.observationLayout().id, std::to_string(b.observationLayout().version),
                    b.actionLayout().id, std::to_string(b.actionLayout().version) };
            })
        .def("action_radians_per_unit", &TrainingBatch3D::actionRadiansPerUnit)
        .def(
            "reset", [](TrainingBatch3D& b, std::uint64_t seed) {
                nb::gil_scoped_release release;
                b.reset(seed);
            },
            "seed"_a)
        .def(
            "step",
            [](TrainingBatch3D& b, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> actions) {
                if (actions.shape(0) != b.environmentCount() || actions.shape(1) != b.actionSize())
                    throw nb::value_error("acoes com forma errada");
                const std::span<const float> data(actions.data(), actions.size());
                nb::gil_scoped_release release;
                b.step(data);
            },
            "actions"_a)
        .def("observations", [](TrainingBatch3D& b) { return view2(b, b.observations(), b.observationSize()); })
        .def("critic_observations",
            [](TrainingBatch3D& b) { return view2(b, b.criticObservations(), b.criticObservationSize()); })
        .def("terminal_critic_observations",
            [](TrainingBatch3D& b) { return view2(b, b.terminalCriticObservations(), b.criticObservationSize()); })
        .def("reward_components",
            [](TrainingBatch3D& b) { return view2(b, b.rewardComponents(), b.rewardComponentCount()); })
        .def("rewards",
            [](TrainingBatch3D& b) {
                return FloatView1(b.rewards().data(), { b.environmentCount() }, nb::find(&b));
            })
        .def("terminated",
            [](TrainingBatch3D& b) {
                return ByteView1(b.terminated().data(), { b.environmentCount() }, nb::find(&b));
            })
        .def("truncated",
            [](TrainingBatch3D& b) {
                return ByteView1(b.truncated().data(), { b.environmentCount() }, nb::find(&b));
            })
        .def("finished_episodes",
            [](TrainingBatch3D& b) {
                // Colunas: mundo, razao, primeiro link no chao, lado da queda,
                // decisoes, segundos, retorno, oscilacao, deriva dos pes,
                // mudanca de acao, esforco, velocidade das juntas, vertical,
                // giro do tronco, giro dos pes no chao.
                const auto episodes = b.finishedEpisodes();
                constexpr std::size_t Columns = 15;
                auto* data = new float[std::max<std::size_t>(1, episodes.size()) * Columns];
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrainingEpisode3D& e = episodes[i];
                    float* row = data + i * Columns;
                    row[0] = static_cast<float>(e.environment);
                    row[1] = static_cast<float>(static_cast<int>(e.reason));
                    row[2] = static_cast<float>(e.firstContactLink);
                    row[3] = static_cast<float>(e.fallDirection);
                    row[4] = static_cast<float>(e.steps);
                    row[5] = e.seconds;
                    row[6] = e.episodeReturn;
                    row[7] = e.comSwayRms;
                    row[8] = e.footDriftMax;
                    row[9] = e.meanActionRate;
                    row[10] = e.meanEffort;
                    row[11] = e.meanJointSpeed;
                    row[12] = e.meanUpright;
                    row[13] = e.trunkTurnRms;
                    row[14] = e.footTurnMax;
                }
                nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, { episodes.size(), Columns }, owner);
            })
        .def("frame_lines", &frameLines, "environment"_a)
        .def(
            "update_task",
            [](TrainingBatch3D& b, const std::string& path) { b.updateTask(loadTrainingTask3D(path)); }, "path"_a)
        .def("last_step_ms", &TrainingBatch3D::lastStepMilliseconds)
        .def("physics_steps", &TrainingBatch3D::physicsStepCount);

    m.attr("END_REASONS") = std::vector<std::string> { "nenhum", "queda_contato", "queda_altura", "numerico", "tempo",
        "limite" };

    nb::class_<PolicyRuntime3D>(m, "Policy")
        .def(nb::init<const std::string&>(), "path"_a)
        .def_prop_ro("obs_size", &PolicyRuntime3D::observationSize)
        .def_prop_ro("action_size", &PolicyRuntime3D::actionSize)
        .def_prop_ro("iteration", [](const PolicyRuntime3D& p) { return p.info().iteration; })
        .def_prop_ro("body_hash", [](const PolicyRuntime3D& p) { return p.info().bodyHash; })
        .def(
            "evaluate",
            [](const PolicyRuntime3D& p, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> obs) {
                if (obs.shape(1) != p.observationSize()) throw nb::value_error("observacao com tamanho errado");
                const std::size_t rows = obs.shape(0);
                auto* data = new float[std::max<std::size_t>(1, rows * p.actionSize())];
                for (std::size_t r = 0; r < rows; ++r)
                    p.evaluate({ obs.data() + r * p.observationSize(), p.observationSize() },
                        { data + r * p.actionSize(), p.actionSize() });
                nb::capsule owner(data, [](void* q) noexcept { delete[] static_cast<float*>(q); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, { rows, p.actionSize() }, owner);
            },
            "observations"_a);
}
