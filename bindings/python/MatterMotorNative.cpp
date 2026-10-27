#include "Engine/RL/Motor/MotorTraining3D.hpp"
#include <algorithm>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace MatterEngine;
namespace {
template <class T> auto owned(std::vector<std::size_t> shape) {
    std::size_t n = 1;
    for (auto s : shape)
        n *= s;
    T *p = new T[std::max<std::size_t>(1, n)]();
    nb::capsule owner(p, [](void *q) noexcept { delete[] static_cast<T *>(q); });
    return nb::ndarray<nb::numpy, T>(p, shape.size(), shape.data(), owner);
}
std::vector<Vec3> coordinates(const MotorTraining3D &b, const std::vector<std::array<float, 3>> &pose) {
    if (pose.size() != b.profile().links.size())
        throw nb::value_error("motor: pose de outro corpo");
    std::vector<Vec3> out;
    for (const auto &p : pose)
        out.push_back({p[0], p[1], p[2]});
    return out;
}
} // namespace
NB_MODULE(matter_motor_native, m) {
    m.def("drill_ids", &MotorTraining3D::drillIds);
    m.def("drill_names", &MotorTraining3D::drillNames);
    m.def("episode_metric_names", [] {
        const auto &names = MotorTraining3D::episodeMetricNames();
        return std::vector<std::string>(names.begin(), names.end());
    });
    nb::class_<MotorTraining3D>(m, "MotorBatch")
        .def(
            "__init__",
            [](MotorTraining3D *b, const std::string &assets, const std::string &task, std::uint32_t worlds,
               std::uint32_t threads, std::uint64_t seed, int drill, bool evaluation, float seconds, float trotSpeed) {
                MotorSettings3D s;
                s.assetsDirectory = assets;
                s.taskPath = task;
                s.environments = worlds;
                s.threads = threads;
                s.seed = seed;
                s.drill = drill;
                s.evaluation = evaluation;
                s.episodeSeconds = seconds;
                s.trotSpeed = trotSpeed;
                nb::gil_scoped_release release;
                new (b) MotorTraining3D(s);
            },
            "assets"_a, "task"_a, "worlds"_a = 512, "threads"_a = 12, "seed"_a = 1, "drill"_a = -1,
            "evaluation"_a = false, "seconds"_a = 20.f, "trot_speed"_a = -1.f)
        .def_prop_ro("num_envs", &MotorTraining3D::environmentCount)
        .def_prop_ro("obs_size", &MotorTraining3D::observationSize)
        .def_prop_ro("action_size", &MotorTraining3D::actionSize)
        .def_prop_ro("imitation_weight", &MotorTraining3D::imitationWeight)
        .def_prop_ro("clip_drive", &MotorTraining3D::clipDrive)
        .def("command_velocity",
             [](MotorTraining3D &b, std::size_t world) {
                 const auto v = b.commandVelocity(world);
                 return std::array<float, 3>{v.x, v.y, v.z};
             })
        .def("identity", &MotorTraining3D::identity)
        .def("observation_names", &MotorTraining3D::observationNames)
        .def("action_names", &MotorTraining3D::actionNames)
        .def("action_derivatives", &MotorTraining3D::actionDerivatives)
        .def("action_mapping", &MotorTraining3D::actionMapping)
        .def("idle_coordinates",
             [](MotorTraining3D &b) {
                 std::vector<std::array<float, 3>> out;
                 for (const auto &v : b.idleCoordinates())
                     out.push_back({v.x, v.y, v.z});
                 return out;
             })
        .def("pose_actions",
             [](MotorTraining3D &b, const std::vector<std::array<float, 3>> &pose) {
                 std::vector<float> out(b.actionSize());
                 b.poseActions(coordinates(b, pose), out);
                 return out;
             })
        .def(
            "place_pose",
            [](MotorTraining3D &b, std::size_t world, const std::vector<std::array<float, 3>> &pose,
               std::array<float, 4> root) { b.placePose(world, coordinates(b, pose), {root[0], root[1], root[2], root[3]}); },
            "world"_a, "pose"_a, "root"_a)
        .def("reset", &MotorTraining3D::reset, "seed"_a)
        .def("set_task", [](MotorTraining3D &b, const std::string &p) { b.setTask(p); })
        .def("set_threads", &MotorTraining3D::setThreads)
        .def("set_trot_limits", [](MotorTraining3D &b, const std::vector<float> &v) { b.setTrotLimits(v); })
        .def("trot_limits", &MotorTraining3D::trotLimits)
        .def("set_shares", [](MotorTraining3D &b, const std::vector<float> &v) { b.setShares(v); })
        .def("shares", &MotorTraining3D::shares)
        .def("look_yaw", &MotorTraining3D::lookYaw)
        .def("set_look", &MotorTraining3D::setLookYaw)
        .def("frame_lines", &MotorTraining3D::frameLines)
        .def("reference_posture_audit", &MotorTraining3D::referencePostureAudit, "trot_speed"_a = 1.f)
        .def("observations",
             [](MotorTraining3D &b) {
                 auto a = owned<float>({b.environmentCount(), b.observationSize()});
                 b.observations({a.data(), a.size()});
                 return a;
             })
        .def("final_observations",
             [](MotorTraining3D &b) {
                 auto a = owned<float>({b.environmentCount(), b.observationSize()});
                 b.observations({a.data(), a.size()}, true);
                 return a;
             })
        .def("reference_actions",
             [](MotorTraining3D &b) {
                 auto a = owned<float>({b.environmentCount(), b.actionSize()});
                 b.referenceActions({a.data(), a.size()});
                 return a;
             })
        .def("step",
             [](MotorTraining3D &b, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> a) {
                 if (a.shape(0) != b.environmentCount() || a.shape(1) != b.actionSize())
                     throw nb::value_error("motor: forma da acao invalida");
                 auto r = owned<float>({b.environmentCount()});
                 auto t = owned<std::uint8_t>({b.environmentCount()}), u = owned<std::uint8_t>({b.environmentCount()});
                 {
                     nb::gil_scoped_release release;
                     b.step({a.data(), a.size()}, {r.data(), r.size()}, {t.data(), t.size()}, {u.data(), u.size()});
                 }
                 return std::make_tuple(r, t, u);
             })
        // Cada linha: mundo, exercicio e as medidas na ordem de episode_metric_names().
        .def("episodes", [](MotorTraining3D &b) {
            const auto episodes = b.takeEpisodes();
            auto a = owned<float>({episodes.size(), 2 + MotorEpisodeMetricCount3D});
            for (std::size_t i = 0; i < episodes.size(); ++i) {
                float *row = a.data() + i * (2 + MotorEpisodeMetricCount3D);
                row[0] = static_cast<float>(episodes[i].world);
                row[1] = static_cast<float>(episodes[i].drill);
                std::copy(episodes[i].values.begin(), episodes[i].values.end(), row + 2);
            }
            return a;
        });
}
