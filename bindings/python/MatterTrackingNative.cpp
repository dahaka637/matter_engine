// A ponte Python do rastreador fisico (DReCon + DeepMimic, vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): o lote de
// treino (o ragdoll do jogo a 0% seguindo o lider, o modo padrao do jogo).
#include <array>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "Engine/Athlete/Training/LocomotionPartnerRunner3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
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
                float star_turning_share, float star_speed, float star_radius, bool real_fall_only, bool lab_leader,
                float follow_slack, const std::string& task_path, const std::string& evaluation_drill,
                float action_filter_beta, float stance_twist_torque, bool leader_side_from_forward,
                float leader_response_percent, float termination_distance, float fall_head_height,
                float capture_step_seconds, float leader_side_amplitude) {
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
                settings.labLeader = lab_leader;
                settings.leaderFollowSlack = follow_slack;
                settings.taskPath = task_path;
                settings.evaluationDrill = evaluation_drill;
                settings.actionFilterBeta = action_filter_beta;
                settings.stanceTwistTorque = stance_twist_torque;
                settings.leaderSideFromForward = leader_side_from_forward;
                settings.leaderResponsePercent = leader_response_percent;
                settings.terminationDistance = termination_distance;
                settings.fallHeadHeight = fall_head_height;
                settings.captureStepSeconds = capture_step_seconds;
                settings.leaderSideAmplitude = leader_side_amplitude;
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
            "star_turning_share"_a = 0.0f, "star_speed"_a = 3.0f, "star_radius"_a = 10.0f, "real_fall_only"_a = false,
            "lab_leader"_a = false, "follow_slack"_a = -1.0f,
            "task_path"_a = "", "evaluation_drill"_a = "", "action_filter_beta"_a = 0.2f,
            "stance_twist_torque"_a = 0.0f, "leader_side_from_forward"_a = false, "leader_response_percent"_a = 0.0f, "termination_distance"_a = 1.0f, "fall_head_height"_a = 0.0f,
            "capture_step_seconds"_a = 0.0f, "leader_side_amplitude"_a = 0.0f)
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
                auto out = owned<float>({ episodes.size(), 13 });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingEpisode3D& e = episodes[i];
                    const float row[13] = { static_cast<float>(e.world), e.seconds, e.fell ? 1.0f : 0.0f, e.meanReward,
                        e.centerOfMassSpeedError, e.bodySpeed, e.requestedSpeed, e.meanHeadDistance,
                        e.fellDown ? 1.0f : 0.0f, e.endHeadHeight, e.relativeDirection, static_cast<float>(e.drill),
                        static_cast<float>(e.taskDrill) };
                    std::copy(row, row + 13, d + i * 13);
                }
                return out;
            })
        .def("drill_ids", &TrackingTraining3D::taskDrillIds)
        .def("look_yaw", &TrackingTraining3D::lookYaw, "world"_a)
        .def("leader_heading", [](const TrackingTraining3D& b, std::size_t world) { return b.leader(world).headingYawRadians; },
            "world"_a)
        .def("leader_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.leader(world).links); },
            "world"_a)
        .def("body_links", [](const TrackingTraining3D& b, std::size_t world) { return links(b.body(world).links); },
            "world"_a)
        // O torque medio do motor de cada junta no ultimo passo e o limite dele, por eixo (links x 6:
        // os 3 torques e os 3 limites, N.m; a raiz em zero): a saturacao dos motores (29/10).
        .def("body_joint_torques",
            [](const TrackingTraining3D& b, std::size_t world) {
                const auto& joints = b.body(world).joints;
                auto out = owned<float>({ joints.size(), 6 });
                float* d = out.data();
                for (std::size_t i = 0; i < joints.size(); ++i)
                    for (int k = 0; k < 3; ++k) {
                        d[i * 6 + k] = joints[i].motorTorqueNewtonMeters[k];
                        d[i * 6 + 3 + k] = joints[i].motorTorqueLimitNewtonMeters[k];
                    }
                return out;
            },
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

    // O levantar livre (31/10, AMP): quedas sorteadas, a nota da tarefa daqui e o estilo pelo discriminador do treinador.
    nb::class_<TrackingGetUpAmpTraining3D>(m, "GetUpAmpBatch")
        .def(
            "__init__",
            [](TrackingGetUpAmpTraining3D* self, const std::string& assets, std::uint32_t environments,
                std::uint32_t threads, std::uint64_t seed, bool evaluation, float episodeSeconds,
                float clipStartFraction, float standingStartFraction, float movingFraction, float avoidFraction,
                std::uint32_t evaluationKind, std::array<float, 2> avoidNewtons, std::array<float, 2> pushNewtons,
                std::array<float, 2> moveSpeed, float sprintStartFraction, float pushHoldFraction,
                std::array<float, 2> pushHoldNewtons, float locomotionFraction, float turnFraction, float walkFraction,
                std::array<float, 2> walkSpeed, std::array<float, 2> trotSpeed, int heading,
                std::array<float, 2> movingPushNewtons, float movingPushHoldFraction,
                std::array<float, 2> movingPushHoldNewtons, float stepReward, bool handoffOnStand,
                float locomotionFallFraction, float partnerSeconds, float partnerFallPenalty, bool partnerAvoid,
                float touchDownPenalty) {
                TrackingGetUpAmpSettings3D settings;
                settings.assetsDirectory = assets;
                settings.environments = environments;
                settings.threads = threads;
                settings.seed = seed;
                settings.evaluation = evaluation;
                settings.episodeSeconds = episodeSeconds;
                settings.clipStartFraction = clipStartFraction;
                settings.standingStartFraction = standingStartFraction;
                settings.movingFraction = movingFraction;
                settings.avoidFraction = avoidFraction;
                settings.evaluationKind = evaluationKind;
                settings.avoidMinimumNewtons = avoidNewtons[0];
                settings.avoidMaximumNewtons = avoidNewtons[1];
                settings.pushMinimumNewtons = pushNewtons[0];
                settings.pushMaximumNewtons = pushNewtons[1];
                settings.moveMinimumSpeed = moveSpeed[0];
                settings.moveMaximumSpeed = moveSpeed[1];
                settings.sprintStartFraction = sprintStartFraction;
                settings.pushHoldFraction = pushHoldFraction;
                settings.pushHoldMinimumNewtons = pushHoldNewtons[0];
                settings.pushHoldMaximumNewtons = pushHoldNewtons[1];
                settings.locomotionFraction = locomotionFraction;
                settings.turnFraction = turnFraction;
                settings.walkFraction = walkFraction;
                settings.walkMinimumSpeed = walkSpeed[0];
                settings.walkMaximumSpeed = walkSpeed[1];
                settings.trotMinimumSpeed = trotSpeed[0];
                settings.trotMaximumSpeed = trotSpeed[1];
                settings.heading = heading;
                settings.movingPushMinimumNewtons = movingPushNewtons[0];
                settings.movingPushMaximumNewtons = movingPushNewtons[1];
                settings.movingPushHoldFraction = movingPushHoldFraction;
                settings.movingPushHoldMinimumNewtons = movingPushHoldNewtons[0];
                settings.movingPushHoldMaximumNewtons = movingPushHoldNewtons[1];
                settings.stepReward = stepReward;
                settings.handoffOnStand = handoffOnStand;
                settings.locomotionFallFraction = locomotionFallFraction;
                settings.partnerSeconds = partnerSeconds;
                settings.partnerFallPenalty = partnerFallPenalty;
                settings.partnerAvoid = partnerAvoid;
                settings.touchDownPenalty = touchDownPenalty;
                nb::gil_scoped_release release;
                new (self) TrackingGetUpAmpTraining3D(settings);
            },
            "assets"_a, "environments"_a = 512, "threads"_a = 0, "seed"_a = 1, "evaluation"_a = false,
            "episode_seconds"_a = 6.0f, "clip_start_fraction"_a = 0.25f, "standing_start_fraction"_a = 0.15f,
            "moving_fraction"_a = 0.0f, "avoid_fraction"_a = 0.0f, "evaluation_kind"_a = 0,
            "avoid_newtons"_a = std::array<float, 2> { 1200.0f, 1800.0f },
            "push_newtons"_a = std::array<float, 2> { 400.0f, 900.0f },
            "move_speed"_a = std::array<float, 2> { 2.5f, 5.0f }, "sprint_start_fraction"_a = 0.0f,
            "push_hold_fraction"_a = 0.0f, "push_hold_newtons"_a = std::array<float, 2> { 150.0f, 350.0f },
            "locomotion_fraction"_a = 0.0f, "turn_fraction"_a = 0.0f, "walk_fraction"_a = 1.0f / 3.0f,
            "walk_speed"_a = std::array<float, 2> { 0.8f, 1.4f }, "trot_speed"_a = std::array<float, 2> { 2.0f, 3.0f },
            "heading"_a = -1, "moving_push_newtons"_a = std::array<float, 2> { 300.0f, 600.0f },
            "moving_push_hold_fraction"_a = 0.3f, "moving_push_hold_newtons"_a = std::array<float, 2> { 100.0f, 180.0f },
            "step_reward"_a = 0.0f, "handoff_on_stand"_a = false, "locomotion_fall_fraction"_a = 0.0f,
            "partner_seconds"_a = 2.0f, "partner_fall_penalty"_a = 10.0f, "partner_avoid"_a = false,
            "touch_down_penalty"_a = 0.0f)
        .def_prop_ro("num_envs", &TrackingGetUpAmpTraining3D::environmentCount)
        // As quedas da locomocao (01/11): [n, links, 13] - a posicao, a orientacao (xyzw) e as velocidades linear e
        // angular de cada link no instante da queda (take_fall_states do lote da locomocao).
        .def(
            "set_fall_starts",
            [](TrackingGetUpAmpTraining3D& b, nb::ndarray<const float, nb::shape<-1, -1, 13>, nb::c_contig> states) {
                std::vector<std::vector<PhysicsBodyState3D>> out(states.shape(0));
                const float* d = states.data();
                for (std::size_t i = 0; i < out.size(); ++i) {
                    out[i].resize(states.shape(1));
                    for (std::size_t k = 0; k < out[i].size(); ++k, d += 13) {
                        PhysicsBodyState3D& s = out[i][k];
                        s.position = { d[0], d[1], d[2] };
                        s.orientation = Quaternion { d[3], d[4], d[5], d[6] };
                        s.linearVelocity = { d[7], d[8], d[9] };
                        s.angularVelocity = { d[10], d[11], d[12] };
                    }
                }
                b.setFallStarts(std::move(out));
            },
            "states"_a)
        .def_prop_ro("fall_start_count", &TrackingGetUpAmpTraining3D::fallStartCount)
        // A rede da locomocao parceira (01/11): a tarefa e a rede exportada do treino dela (vazio: sem). Devolve a rodada
        // da rede.
        .def(
            "set_locomotion_partner",
            [](TrackingGetUpAmpTraining3D& b, const std::string& assets, const std::string& task,
                const std::string& policy) -> std::uint64_t {
                if (policy.empty()) {
                    b.setLocomotionPartner(nullptr);
                    return 0;
                }
                auto partner = std::make_shared<LocomotionPartnerRunner3D>(assets, task, policy);
                const std::uint64_t iteration = partner->iteration();
                b.setLocomotionPartner(std::move(partner));
                return iteration;
            },
            "assets"_a, "task"_a, "policy"_a)
        .def_prop_ro("obs_size", &TrackingGetUpAmpTraining3D::observationSize)
        .def_prop_ro("action_size", &TrackingGetUpAmpTraining3D::actionSize)
        .def_prop_ro("amp_size", &TrackingGetUpAmpTraining3D::ampSize)
        .def_prop_ro("episode_seconds", &TrackingGetUpAmpTraining3D::episodeSeconds)
        .def_prop_ro("policy_dt", [](const TrackingGetUpAmpTraining3D&) {
            return TrackingGetUpAmpTraining3D::TicksPerDecision * TrackingGetUpAmpTraining3D::TickSeconds;
        })
        .def("action_ranges",
            [](const TrackingGetUpAmpTraining3D& b) {
                const std::vector<float> ranges = b.actionRanges();
                auto out = owned<float>({ ranges.size() / 2, 2 });
                std::copy(ranges.begin(), ranges.end(), out.data());
                return out;
            })
        .def("observation_names", &TrackingGetUpAmpTraining3D::observationNames)
        .def("action_names", &TrackingGetUpAmpTraining3D::actionNames)
        .def(
            "reset",
            [](TrackingGetUpAmpTraining3D& b, std::uint64_t seed) {
                nb::gil_scoped_release release;
                b.reset(seed);
            },
            "seed"_a)
        .def("observations",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.observations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("final_observations",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.observationSize() });
                b.finalObservations(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("amp_pairs",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), 2 * b.ampSize() });
                b.ampPairs(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("reference_pairs",
            [](const TrackingGetUpAmpTraining3D& b) {
                std::vector<float> pairs;
                {
                    nb::gil_scoped_release release;
                    pairs = b.referencePairs();
                }
                const std::size_t width = 2 * b.ampSize();
                auto out = owned<float>({ pairs.size() / width, width });
                std::copy(pairs.begin(), pairs.end(), out.data());
                return out;
            })
        .def(
            "step",
            [](TrackingGetUpAmpTraining3D& b,
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
            [](const TrackingGetUpAmpTraining3D&) {
                return std::vector<std::string> { "mundo", "comeco", "lado", "cabeca_inicio", "segundos", "de_pe_aos",
                    "nota_tarefa", "altura_cabeca", "altura_pelve", "de_pe", "velocidade_pedida", "chegou_aos",
                    "velocidade_rumo_alvo", "caiu_depois_de_pe", "recompos_aos", "pose_fim", "erro_rumo_graus", "coluna_fim", "troca_aos",
                    "parceiro_caiu", "parceiro_s", "tocou_chao" };
            })
        .def("finished_episodes",
            [](TrackingGetUpAmpTraining3D& b) {
                const std::vector<TrackingGetUpAmpEpisode3D> episodes = b.takeEpisodes();
                constexpr std::size_t width = 22;
                auto out = owned<float>({ episodes.size(), width });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingGetUpAmpEpisode3D& e = episodes[i];
                    const float row[width] = { static_cast<float>(e.world), static_cast<float>(e.startKind),
                        static_cast<float>(e.startSide), e.startHeadHeight, e.seconds, e.firstStandSeconds,
                        e.meanTaskReward, e.headHeight, e.pelvisHeight, e.standing ? 1.0f : 0.0f, e.commandSpeed,
                        e.reachedSeconds, e.meanTowardSpeed, e.fellAfterStanding ? 1.0f : 0.0f, e.recomposedSeconds,
                        e.endPoseMatch, e.meanFaceError, e.endSpineMatch, e.handoffSeconds, e.partnerFell ? 1.0f : 0.0f,
                        e.partnerSeconds, e.touchedDown ? 1.0f : 0.0f };
                    std::copy(row, row + width, d + i * width);
                }
                return out;
            })
        .def("frame_lines", &TrackingGetUpAmpTraining3D::frameLines, "world"_a);
}
