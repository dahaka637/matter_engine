// A ponte Python do treino da locomocao (a etapa B do plano do RL, vault
// 04_locomocao/plano_fases_1_2.md): o lote de RL sobre o atleta do jogo, com
// o contrato v2. Um modulo separado do matter_rl_native (o RL do zero, parado):
// os dois tem um ReferenceMotion3D diferente.
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/vector.h>

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/Training/LocomotionTraining3D.hpp"

#include <cstdio>
#include <span>
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

FloatView2 view2(LocomotionTraining3D& batch, std::span<const float> data, std::size_t columns) {
    return FloatView2(data.data(), { batch.environmentCount(), columns }, nb::find(&batch));
}

nb::dict mirrorMap(const LocomotionMirrorMap3D& map) {
    nb::dict out;
    out["source"] = map.source;
    out["scale"] = map.scale;
    out["offset"] = map.offset;
    return out;
}

} // namespace

NB_MODULE(matter_locomotion_native, m) {
    m.doc() = "O treino da locomocao do Matter Engine (o atleta do jogo, sem janela)";

    nb::class_<LocomotionTraining3D>(m, "LocomotionBatch")
        .def(
            "__init__",
            [](LocomotionTraining3D* self, const std::string& task, const std::string& assets, std::uint32_t environments,
                std::uint32_t threads, std::uint64_t seed, bool evaluation, float evaluationSeconds,
                const std::string& drill) {
                LocomotionTrainingSettings3D settings;
                settings.taskPath = task;
                settings.assetsDirectory = assets;
                settings.environments = environments;
                settings.threads = threads;
                settings.seed = seed;
                settings.evaluation = evaluation;
                settings.evaluationSeconds = evaluationSeconds;
                settings.drill = drill;
                nb::gil_scoped_release release;
                new (self) LocomotionTraining3D(settings);
            },
            "task"_a, "assets"_a, "environments"_a = 256, "threads"_a = 0, "seed"_a = 1, "evaluation"_a = false,
            "evaluation_seconds"_a = 20.0f, "drill"_a = "")
        .def_prop_ro("num_envs", &LocomotionTraining3D::environmentCount)
        .def_prop_ro("obs_size", &LocomotionTraining3D::observationSize)
        .def_prop_ro("critic_size", &LocomotionTraining3D::criticObservationSize)
        .def_prop_ro("action_size", &LocomotionTraining3D::actionSize)
        .def_prop_ro("reward_count", [](const LocomotionTraining3D&) { return LocomotionRewardTermCount3D; })
        .def_prop_ro("threads", &LocomotionTraining3D::workerThreads)
        .def_prop_ro("policy_dt", &LocomotionTraining3D::policyStepSeconds)
        .def_prop_ro("contract_version", [](const LocomotionTraining3D&) { return LocomotionContract3D::Version; })
        .def_prop_ro("contract_hash", [](const LocomotionTraining3D& b) { return hex(b.contract().hash()); })
        .def_prop_ro("body_id", [](const LocomotionTraining3D& b) { return b.athlete().profile().id; })
        // Os corpos que os mundos sorteiam ((altura m, massa kg); sem o bloco "body", so o de hoje).
        .def_prop_ro("body_grid",
            [](const LocomotionTraining3D& b) {
                std::vector<std::pair<float, float>> grid;
                for (const PlayerBody3D& body : b.bodyGrid()) grid.emplace_back(body.heightMeters, body.massKg);
                return grid;
            })
        .def_prop_ro("task_id", [](const LocomotionTraining3D& b) { return b.task().id; })
        .def_prop_ro("task_name", [](const LocomotionTraining3D& b) { return b.task().displayName; })
        .def_prop_ro("task_hash", [](const LocomotionTraining3D& b) { return hex(b.task().hash); })
        .def_prop_ro("drill_ids",
            [](const LocomotionTraining3D& b) {
                std::vector<std::string> ids;
                for (const LocomotionDrill3D& drill : b.task().drills) ids.push_back(drill.id);
                return ids;
            })
        .def_prop_ro("drill_phases",
            [](const LocomotionTraining3D& b) {
                std::vector<int> phases;
                for (const LocomotionDrill3D& drill : b.task().drills) phases.push_back(drill.phase);
                return phases;
            })
        .def("drill_minimums",
            [](const LocomotionTraining3D& b) {
                nb::dict out;
                for (const LocomotionDrill3D& drill : b.task().drills) {
                    nb::dict minimum;
                    minimum["fall_fraction"] = drill.maximumFallFraction;
                    minimum["velocity_error"] = drill.maximumVelocityError;
                    out[drill.id.c_str()] = minimum;
                }
                return out;
            })
        .def("world_drills",
            [](LocomotionTraining3D& b) {
                return nb::ndarray<nb::numpy, const std::uint32_t, nb::ndim<1>>(
                    b.worldDrills().data(), { b.environmentCount() }, nb::find(&b));
            })
        .def("observation_names", [](const LocomotionTraining3D& b) { return names(b.contract().observationLayout()); })
        .def("critic_names", [](const LocomotionTraining3D& b) { return names(b.criticLayout()); })
        .def("action_names", [](const LocomotionTraining3D& b) { return names(b.contract().actionLayout()); })
        .def("reward_names",
            [](const LocomotionTraining3D&) {
                return std::vector<std::string>(LocomotionRewardTermNames3D.begin(), LocomotionRewardTermNames3D.end());
            })
        .def_prop_ro("reward_positive_terms", [](const LocomotionTraining3D&) { return LocomotionRewardPositiveTerms3D; })
        .def("episode_columns", [](const LocomotionTraining3D&) { return locomotionEpisodeColumns3D(); })
        .def("end_reasons",
            [](const LocomotionTraining3D&) {
                std::vector<std::string> reasons;
                for (int r = 0; r <= static_cast<int>(LocomotionEnd3D::Failure); ++r)
                    reasons.push_back(locomotionEndName3D(static_cast<LocomotionEnd3D>(r)));
                return reasons;
            })
        .def("mirror_maps",
            [](const LocomotionTraining3D& b) {
                const LocomotionTraining3D::Mirror mirror = b.mirror();
                nb::dict out;
                out["observation"] = mirrorMap(mirror.observation);
                out["critic"] = mirrorMap(mirror.critic);
                out["action"] = mirrorMap(mirror.action);
                return out;
            })
        .def("action_cap_fractions", &LocomotionTraining3D::actionCapFractions)
        .def("action_scales", &LocomotionTraining3D::actionScales)
        .def("action_bounds", [](const LocomotionTraining3D& self) {
            const LocomotionTraining3D::ActionBounds bounds = self.actionBounds();
            return std::make_pair(bounds.lower, bounds.upper);
        })
        .def("saved_start_count", &LocomotionTraining3D::savedStartCount)
        .def(
            "set_authority",
            [](LocomotionTraining3D& b, float footwork, float balance, float pose, float gain) {
                b.setAuthority(footwork, balance, pose, gain);
            },
            "footwork"_a, "balance"_a, "pose"_a, "gain"_a)
        .def(
            "set_turn_move", [](LocomotionTraining3D& b, int mode) { b.setTurnMoveMode(mode); }, "mode"_a)
        .def(
            "set_drill_weights",
            [](LocomotionTraining3D& b, const std::vector<float>& weights) { b.setDrillWeights(weights); }, "weights"_a)
        .def(
            "update_task", [](LocomotionTraining3D& b, const std::string& path) { b.updateTask(loadLocomotionTask3D(path)); },
            "path"_a)
        .def(
            "reset",
            [](LocomotionTraining3D& b, std::uint64_t seed) {
                nb::gil_scoped_release release;
                b.reset(seed);
            },
            "seed"_a)
        .def(
            "step",
            [](LocomotionTraining3D& b, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> actions) {
                if (actions.shape(0) != b.environmentCount() || actions.shape(1) != b.actionSize())
                    throw nb::value_error("acoes com forma errada");
                const std::span<const float> a(actions.data(), actions.size());
                nb::gil_scoped_release release;
                b.step(a);
            },
            "actions"_a)
        .def("observations", [](LocomotionTraining3D& b) { return view2(b, b.observations(), b.observationSize()); })
        .def("critic_observations",
            [](LocomotionTraining3D& b) { return view2(b, b.criticObservations(), b.criticObservationSize()); })
        .def("terminal_critic_observations",
            [](LocomotionTraining3D& b) { return view2(b, b.terminalCriticObservations(), b.criticObservationSize()); })
        .def("reward_components",
            [](LocomotionTraining3D& b) { return view2(b, b.rewardTerms(), LocomotionRewardTermCount3D); })
        .def("rewards",
            [](LocomotionTraining3D& b) { return FloatView1(b.rewards().data(), { b.environmentCount() }, nb::find(&b)); })
        .def("terminated",
            [](LocomotionTraining3D& b) { return ByteView1(b.terminated().data(), { b.environmentCount() }, nb::find(&b)); })
        .def("truncated",
            [](LocomotionTraining3D& b) { return ByteView1(b.truncated().data(), { b.environmentCount() }, nb::find(&b)); })
        .def("finished_episodes",
            [](LocomotionTraining3D& b) {
                const auto& episodes = b.finishedEpisodes();
                const std::size_t columns = locomotionEpisodeColumns3D().size();
                auto* data = new float[std::max<std::size_t>(1, episodes.size()) * columns];
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const std::vector<float> row = locomotionEpisodeRow3D(episodes[i]);
                    std::copy(row.begin(), row.end(), data + i * columns);
                }
                nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, { episodes.size(), columns }, owner);
            })
        .def("frame_lines", &LocomotionTraining3D::frameLines, "environment"_a)
        .def("last_step_ms", &LocomotionTraining3D::lastStepMilliseconds);
}
