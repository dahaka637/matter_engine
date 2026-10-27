// A ponte Python do rastreador fisico (DReCon + DeepMimic, vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): o lote de
// treino (o ragdoll do jogo a 0% seguindo o lider, o modo padrao do jogo).
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "Engine/Tracking/TrackingGetUpTraining3D.hpp"
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
                float switch_seconds, bool projectiles, float projectile_minimum_kg, float projectile_maximum_kg,
                float idle_share, float turn_share, float fast_turn_share, float action_rate_weight,
                bool adaptive_legs, float leg_weight, float center_of_mass_velocity_weight, float arm_weight,
                float trunk_weight, float alive_bonus, float speed_scale, float star_share, float star_fixed_share,
                float star_turning_share, float star_speed, float star_radius, bool real_fall_only) {
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
                else if (evaluation_mode == "parado") settings.evaluationMode = TrackingEvaluation3D::Idle;
                else if (evaluation_mode == "giro") settings.evaluationMode = TrackingEvaluation3D::Turn;
                else if (evaluation_mode == "giro_rapido") settings.evaluationMode = TrackingEvaluation3D::FastTurn;
                else if (evaluation_mode == "estrela") settings.evaluationMode = TrackingEvaluation3D::Star;
                else if (evaluation_mode == "estrela_fixo") settings.evaluationMode = TrackingEvaluation3D::StarFixed;
                else if (evaluation_mode == "estrela_girando") settings.evaluationMode = TrackingEvaluation3D::StarTurning;
                else throw nb::value_error("avaliacao: 'frente', 'rumos', 'trocas', 'parado', 'giro', 'giro_rapido', 'estrela', 'estrela_fixo' ou 'estrela_girando'");
                settings.starShare = star_share;
                settings.starFixedShare = star_fixed_share;
                settings.starTurningShare = star_turning_share;
                settings.starSpeed = star_speed;
                settings.starRadius = star_radius;
                settings.realFallOnly = real_fall_only;
                settings.idleShare = idle_share;
                settings.turnShare = turn_share;
                settings.fastTurnShare = fast_turn_share;
                settings.actionRateWeight = action_rate_weight;
                settings.adaptiveLegs = adaptive_legs;
                settings.legWeight = leg_weight;
                settings.centerOfMassVelocityWeight = center_of_mass_velocity_weight;
                settings.armWeight = arm_weight;
                settings.trunkWeight = trunk_weight;
                settings.aliveBonus = alive_bonus;
                settings.speedScale = speed_scale;
                settings.switchSeconds = switch_seconds;
                settings.projectiles = projectiles;
                settings.projectileMinimumKg = projectile_minimum_kg;
                settings.projectileMaximumKg = projectile_maximum_kg;
                nb::gil_scoped_release release;
                new (self) TrackingTraining3D(settings);
            },
            "assets"_a, "environments"_a = 512, "threads"_a = 0, "seed"_a = 1, "scenario"_a = "frente",
            "forward_maximum_speed"_a = 3.0f, "evaluation"_a = false, "evaluation_speed"_a = 3.0f,
            "episode_seconds"_a = 20.0f, "allow_sprint"_a = true, "evaluation_mode"_a = "frente",
            "switch_seconds"_a = 4.0f, "projectiles"_a = false, "projectile_minimum_kg"_a = 0.01f,
            "projectile_maximum_kg"_a = 8.0f, "idle_share"_a = 0.0f, "turn_share"_a = 0.0f, "fast_turn_share"_a = 0.0f,
            "action_rate_weight"_a = 0.0f, "adaptive_legs"_a = false, "leg_weight"_a = 1.0f,
            "center_of_mass_velocity_weight"_a = 1.0f, "arm_weight"_a = 1.0f, "trunk_weight"_a = 1.0f,
            "alive_bonus"_a = 0.0f, "speed_scale"_a = 1.0f, "star_share"_a = 0.0f, "star_fixed_share"_a = 0.0f,
            "star_turning_share"_a = 0.0f, "star_speed"_a = 3.0f, "star_radius"_a = 10.0f, "real_fall_only"_a = false)
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
                    "vel_corpo", "vel_pedida", "dist_cabeca", "caiu_de_verdade", "altura_cabeca_fim", "rumo", "exercicio" };
            })
        .def("finished_episodes",
            [](TrackingTraining3D& b) {
                const std::vector<TrackingEpisode3D> episodes = b.takeEpisodes();
                auto out = owned<float>({ episodes.size(), 12 });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingEpisode3D& e = episodes[i];
                    const float row[12] = { static_cast<float>(e.world), e.seconds, e.fell ? 1.0f : 0.0f, e.meanReward,
                        e.centerOfMassSpeedError, e.bodySpeed, e.requestedSpeed, e.meanHeadDistance,
                        e.fellDown ? 1.0f : 0.0f, e.endHeadHeight, e.relativeDirection, static_cast<float>(e.drill) };
                    std::copy(row, row + 12, d + i * 12);
                }
                return out;
            })
        .def("look_yaw", &TrackingTraining3D::lookYaw, "world"_a)
        .def("leader_heading", [](const TrackingTraining3D& b, std::size_t world) { return b.leader(world).headingYawRadians; },
            "world"_a)
        .def("leader_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.leader(world).links); },
            "world"_a)
        .def("body_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.body(world).links); },
            "world"_a)
        .def("frame_lines", &TrackingTraining3D::frameLines, "world"_a, "leader"_a = false);

    // O levantar sem ajuda (DeepMimic): o ragdoll do jogo imitando o clipe de levantar.
    nb::class_<TrackingGetUpTraining3D>(m, "GetUpBatch")
        .def(
            "__init__",
            [](TrackingGetUpTraining3D* self, const std::string& assets, const std::string& clip,
                std::uint32_t environments, std::uint32_t threads, std::uint64_t seed, bool evaluation) {
                TrackingGetUpSettings3D settings;
                settings.assetsDirectory = assets;
                settings.clip = clip;
                settings.environments = environments;
                settings.threads = threads;
                settings.seed = seed;
                settings.evaluation = evaluation;
                nb::gil_scoped_release release;
                new (self) TrackingGetUpTraining3D(settings);
            },
            "assets"_a, "clip"_a = "frente", "environments"_a = 512, "threads"_a = 0, "seed"_a = 1,
            "evaluation"_a = false)
        .def_prop_ro("num_envs", &TrackingGetUpTraining3D::environmentCount)
        .def_prop_ro("obs_size", &TrackingGetUpTraining3D::observationSize)
        .def_prop_ro("action_size", &TrackingGetUpTraining3D::actionSize)
        .def_prop_ro("clip_seconds", &TrackingGetUpTraining3D::clipSeconds)
        .def_prop_ro("policy_dt", [](const TrackingGetUpTraining3D&) {
            return TrackingGetUpTraining3D::TicksPerDecision * TrackingGetUpTraining3D::TickSeconds;
        })
        .def("observation_names", &TrackingGetUpTraining3D::observationNames)
        .def("action_names", &TrackingGetUpTraining3D::actionNames)
        .def(
            "reset",
            [](TrackingGetUpTraining3D& b, std::uint64_t seed) {
                nb::gil_scoped_release release;
                b.reset(seed);
            },
            "seed"_a)
        .def("observations",
            [](const TrackingGetUpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.observations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("final_observations",
            [](const TrackingGetUpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.finalObservations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("reference_actions",
            [](const TrackingGetUpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.actionSize() });
                b.referenceActions(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def(
            "step",
            [](TrackingGetUpTraining3D& b,
                nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> actions) {
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
            [](const TrackingGetUpTraining3D&) {
                return std::vector<std::string> { "mundo", "inicio", "segundos", "nota_media", "altura_cabeca",
                    "altura_pelve", "de_pe" };
            })
        .def("finished_episodes",
            [](TrackingGetUpTraining3D& b) {
                const std::vector<TrackingGetUpEpisode3D> episodes = b.takeEpisodes();
                auto out = owned<float>({ episodes.size(), 7 });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingGetUpEpisode3D& e = episodes[i];
                    const float row[7] = { static_cast<float>(e.world), e.startSeconds, e.seconds, e.meanReward,
                        e.headHeight, e.pelvisHeight, e.standing ? 1.0f : 0.0f };
                    std::copy(row, row + 7, d + i * 7);
                }
                return out;
            })
        .def("frame_lines", &TrackingGetUpTraining3D::frameLines, "world"_a, "reference"_a = false);
}
