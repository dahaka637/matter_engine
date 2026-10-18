// A ponte Python do rastreador fisico (DReCon + DeepMimic, vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): o lote de
// treino (o ragdoll do jogo a 0% seguindo o lider, o modo padrao do jogo).
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "Engine/Tracking/TrackingTraining3D.hpp"

#include <algorithm>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;
using namespace MatterEngine;

namespace {

template <typename T>
nb::ndarray<nb::numpy, T> owned(std::vector<std::size_t> shape) {
    std::size_t count = 1;
    for (std::size_t s : shape) count *= s;
    T* data = new T[std::max<std::size_t>(1, count)]();
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<T*>(p); });
    return nb::ndarray<nb::numpy, T>(data, shape.size(), shape.data(), owner);
}

// Os links no mundo (posicao xyz e orientacao xyzw), um por linha.
nb::ndarray<nb::numpy, float> links(std::span<const PhysicsBodyState3D> states) {
    auto out = owned<float>({ states.size(), 7 });
    float* d = out.data();
    for (std::size_t i = 0; i < states.size(); ++i) {
        const PhysicsBodyState3D& s = states[i];
        const float row[7] = { s.position.x, s.position.y, s.position.z, s.orientation.x, s.orientation.y,
            s.orientation.z, s.orientation.w };
        std::copy(row, row + 7, d + i * 7);
    }
    return out;
}

} // namespace

NB_MODULE(matter_tracking_native, m) {
    m.doc() = "O rastreador fisico do Matter Engine (o ragdoll do jogo seguindo o modo padrao, sem janela)";

    nb::class_<TrackingTraining3D>(m, "TrackingBatch")
        .def(
            "__init__",
            [](TrackingTraining3D* self, const std::string& assets, std::uint32_t environments, std::uint32_t threads,
                std::uint64_t seed, const std::string& scenario, float forward_maximum_speed, bool evaluation,
                float evaluation_speed, float episode_seconds, bool allow_sprint, const std::string& evaluation_mode,
                float switch_seconds) {
                TrackingTrainingSettings3D settings;
                settings.assetsDirectory = assets;
                settings.environments = environments;
                settings.threads = threads;
                settings.seed = seed;
                if (scenario == "frente") settings.scenario = TrackingScenario3D::Forward;
                else if (scenario == "completo") settings.scenario = TrackingScenario3D::Full;
                else throw nb::value_error("cenario: 'frente' ou 'completo'");
                settings.forwardMaximumSpeed = forward_maximum_speed;
                settings.evaluation = evaluation;
                settings.evaluationSpeed = evaluation_speed;
                settings.episodeSeconds = episode_seconds;
                settings.allowSprint = allow_sprint;
                if (evaluation_mode == "frente") settings.evaluationMode = TrackingEvaluation3D::Forward;
                else if (evaluation_mode == "rumos") settings.evaluationMode = TrackingEvaluation3D::Headings;
                else if (evaluation_mode == "trocas") settings.evaluationMode = TrackingEvaluation3D::Switches;
                else throw nb::value_error("avaliacao: 'frente', 'rumos' ou 'trocas'");
                settings.switchSeconds = switch_seconds;
                nb::gil_scoped_release release;
                new (self) TrackingTraining3D(settings);
            },
            "assets"_a, "environments"_a = 512, "threads"_a = 0, "seed"_a = 1, "scenario"_a = "frente",
            "forward_maximum_speed"_a = 3.0f, "evaluation"_a = false, "evaluation_speed"_a = 3.0f,
            "episode_seconds"_a = 20.0f, "allow_sprint"_a = true, "evaluation_mode"_a = "frente",
            "switch_seconds"_a = 4.0f)
        .def_prop_ro("num_envs", &TrackingTraining3D::environmentCount)
        .def_prop_ro("obs_size", &TrackingTraining3D::observationSize)
        .def_prop_ro("action_size", &TrackingTraining3D::actionSize)
        .def_prop_ro("threads", &TrackingTraining3D::workerThreads)
        .def_prop_ro("policy_dt", [](const TrackingTraining3D&) {
            return TrackingTraining3D::TicksPerDecision * TrackingTraining3D::TickSeconds;
        })
        .def("observation_names", &TrackingTraining3D::observationNames)
        .def("action_names", &TrackingTraining3D::actionNames)
        .def("link_names",
            [](const TrackingTraining3D& b) {
                std::vector<std::string> names;
                for (const RagdollLinkDefinition3D& link : b.profile().links) names.push_back(link.id);
                return names;
            })
        .def(
            "reset",
            [](TrackingTraining3D& b, std::uint64_t seed) {
                nb::gil_scoped_release release;
                b.reset(seed);
            },
            "seed"_a)
        .def("observations",
            [](const TrackingTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.observations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("final_observations",
            [](const TrackingTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.finalObservations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def(
            "step",
            [](TrackingTraining3D& b, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> actions) {
                const std::size_t n = b.environmentCount();
                if (actions.shape(0) != n || actions.shape(1) != b.actionSize())
                    throw nb::value_error("acoes com forma errada");
                auto rewards = owned<float>({ n });
                auto terminated = owned<std::uint8_t>({ n });
                auto truncated = owned<std::uint8_t>({ n });
                {
                    const std::span<const float> a(actions.data(), actions.size());
                    nb::gil_scoped_release release;
                    b.step(a, std::span<float>(rewards.data(), n), std::span<std::uint8_t>(terminated.data(), n),
                        std::span<std::uint8_t>(truncated.data(), n));
                }
                return std::make_tuple(rewards, terminated, truncated);
            },
            "actions"_a)
        .def("episode_columns",
            [](const TrackingTraining3D&) {
                return std::vector<std::string> { "mundo", "segundos", "caiu", "nota_media", "erro_vel_cm",
                    "vel_corpo", "vel_pedida", "dist_cabeca", "caiu_de_verdade", "altura_cabeca_fim" };
            })
        .def("finished_episodes",
            [](TrackingTraining3D& b) {
                const std::vector<TrackingEpisode3D> episodes = b.takeEpisodes();
                auto out = owned<float>({ episodes.size(), 10 });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingEpisode3D& e = episodes[i];
                    const float row[10] = { static_cast<float>(e.world), e.seconds, e.fell ? 1.0f : 0.0f, e.meanReward,
                        e.centerOfMassSpeedError, e.bodySpeed, e.requestedSpeed, e.meanHeadDistance,
                        e.fellDown ? 1.0f : 0.0f, e.endHeadHeight };
                    std::copy(row, row + 10, d + i * 10);
                }
                return out;
            })
        .def("leader_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.leader(world).links); },
            "world"_a)
        .def("body_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.body(world).links); },
            "world"_a)
        .def("frame_lines", &TrackingTraining3D::frameLines, "world"_a, "leader"_a = false);
}
