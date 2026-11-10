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
                float touchDownPenalty, float supportBonus, float flexReward, float obstacleFraction,
                float weightFraction, float runFraction, float sprintFraction, float locomotionClipFraction,
                float locomotionSeconds, bool locomotionTermination,
                int evaluationBand, float speedRampPower, float imitationWeight, const std::string& onlyClip, float clipBlend,
                bool protectiveFall, bool getUpGuide, float getUpSeconds, float rollBlend, std::array<float, 6> scenery, float avoidSceneryFraction, bool evaluationScenery, bool separateSkills, float trackingTermination, bool locomotionResidual, bool taughtHeadings, bool balanceObservation, bool stepChannels) {
                TrackingGetUpAmpSettings3D settings;
                settings.balanceObservation = balanceObservation;
                settings.stepChannels = stepChannels;
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
                settings.supportBonus = supportBonus;
                settings.flexReward = flexReward;
                settings.obstacleFraction = obstacleFraction;
                settings.weightFraction = weightFraction;
                settings.runFraction = runFraction;
                settings.sprintFraction = sprintFraction;
                settings.locomotionClipFraction = locomotionClipFraction;
                settings.locomotionSeconds = locomotionSeconds;
                settings.locomotionTermination = locomotionTermination;
                settings.evaluationBand = evaluationBand;
                settings.speedRampPower = speedRampPower;
                settings.imitationWeight = imitationWeight;
                settings.onlyClip = onlyClip;
                settings.clipBlend = clipBlend;
                settings.protectiveFall = protectiveFall;
                settings.getUpGuide = getUpGuide;
                settings.getUpSeconds = getUpSeconds;
                settings.rollBlend = rollBlend;
                // (Os ambientes: corredor, pilares, rampa, acidentado, corpos embaixo, corpo em cima.)
                settings.corridorFraction = scenery[0];
                settings.pillarFraction = scenery[1];
                settings.slopeFraction = scenery[2];
                settings.roughFraction = scenery[3];
                settings.bodiesUnderFraction = scenery[4];
                settings.bodyOnTopFraction = scenery[5];
                settings.avoidSceneryFraction = avoidSceneryFraction;
                settings.evaluationScenery = evaluationScenery;
                settings.separateSkills = separateSkills;
                settings.trackingTermination = trackingTermination;
                settings.locomotionResidual = locomotionResidual;
                settings.taughtHeadings = taughtHeadings;
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
            "touch_down_penalty"_a = 0.0f, "support_bonus"_a = 0.0f, "flex_reward"_a = 0.0f,
            "obstacle_fraction"_a = 0.0f, "weight_fraction"_a = 0.0f, "run_fraction"_a = 0.0f,
            "sprint_fraction"_a = 0.0f, "locomotion_clip_fraction"_a = 0.5f, "locomotion_seconds"_a = 0.0f,
            "locomotion_termination"_a = false,
            "evaluation_band"_a = -1, "speed_ramp_power"_a = 0.5f, "imitation_weight"_a = 0.0f, "only_clip"_a = std::string(), "clip_blend"_a = 1.0f,
            "protective_fall"_a = false, "get_up_guide"_a = false, "get_up_seconds"_a = 1.7f, "roll_blend"_a = 1.0f,
            "scenery"_a = std::array<float, 6> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, "avoid_scenery_fraction"_a = 0.0f, "evaluation_scenery"_a = false, "separate_skills"_a = false, "tracking_termination"_a = 0.0f, "locomotion_residual"_a = false, "taught_headings"_a = false, "balance_observation"_a = false, "step_channels"_a = false)
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
        // Os mundos protegendo-se na queda agora (1 ou 0; o alivio dos custos de suavidade no treinador).
        .def("protecting",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<std::uint8_t>({ b.environmentCount() });
                b.protecting(std::span<std::uint8_t>(out.data(), b.environmentCount()));
                return out;
            })
        // Os mundos num episodio de locomocao (1 ou 0; os pesos da locomocao no treinador, a rl_final).
        .def("locomoting",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<std::uint8_t>({ b.environmentCount() });
                b.locomoting(std::span<std::uint8_t>(out.data(), b.environmentCount()));
                return out;
            })
        // O rastreio da ultima decisao (03/11): a nota, os erros da pelve, das pernas, do tronco, dos bracos, dos pes, da
        // velocidade e a velocidade no rumo pedido, por mundo.
        .def("locomotion_feedback",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), TrackingGetUpAmpTraining3D::TrackingFeedbackSize });
                b.locomotionFeedback(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("locomotion_targets", [](const TrackingGetUpAmpTraining3D& b) {
            auto out = owned<float>({ b.environmentCount(), b.actionSize() });
            b.locomotionTargets(std::span<float>(out.data(), out.size()));
            return out;
        })
        // (06/11) A professora da locomocao livre: a correcao dela e quem ela dirige, antes do step; os rotulos, depois.
        .def("set_self_teacher",
            [](TrackingGetUpAmpTraining3D& b, nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> actions,
                nb::ndarray<const float, nb::ndim<1>, nb::c_contig, nb::device::cpu> share) {
                if (actions.shape(0) != b.environmentCount() || actions.shape(1) != b.actionSize() || share.shape(0) != b.environmentCount())
                    throw nb::value_error("professora com forma errada");
                b.setSelfTeacher(std::span<const float>(actions.data(), actions.size()),
                    std::span<const float>(share.data(), share.size()));
            }, "actions"_a, "share"_a)
        .def("clear_self_teacher", [](TrackingGetUpAmpTraining3D& b) { b.setSelfTeacher({}, {}); })
        .def("self_teacher_labels", [](const TrackingGetUpAmpTraining3D& b) {
            auto out = owned<float>({ b.environmentCount(), b.actionSize() + 1 });
            b.selfTeacherLabels(std::span<float>(out.data(), out.size()));
            return out;
        })
        // (05/11) O espelho de cada mundo: [vale, deslocamento da fase] (a perda de simetria; ler antes do step).
        .def("mirror_info", [](const TrackingGetUpAmpTraining3D& b) {
            auto out = owned<float>({ b.environmentCount(), TrackingGetUpAmpTraining3D::MirrorInfoSize });
            b.mirrorInfo(std::span<float>(out.data(), out.size()));
            return out;
        })
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
        // A professora da locomocao (03/11 18h27, a B21 nos 8 rumos): a tarefa e a rede exportada dela (vazio: sem), a fatia
        // dos rumos com ela no volante (beta), os pedidos por rumo (a ela e a aluna, m/s) e a intervencao (tronco graus,
        // tronco graus/s, peito graus, rumo graus, segundos). Devolve a rodada da rede; vale nos proximos comecos.
        .def(
            "set_locomotion_teacher",
            [](TrackingGetUpAmpTraining3D& b, const std::string& assets, const std::string& task, const std::string& policy,
                float share, std::array<float, 8> speeds, std::array<float, 8> studentSpeeds,
                std::array<float, 5> takeover) -> std::uint64_t {
                b.setTeacherSettings(speeds, studentSpeeds, takeover);
                b.setTeacherShare(share);
                if (policy.empty()) {
                    b.setLocomotionTeacher(nullptr);
                    return 0;
                }
                auto teacher = std::make_shared<LocomotionPartnerRunner3D>(assets, task, policy);
                const std::uint64_t iteration = teacher->iteration();
                b.setLocomotionTeacher(std::move(teacher));
                return iteration;
            },
            "assets"_a, "task"_a, "policy"_a, "share"_a = 1.0f,
            "speeds"_a = std::array<float, 8> { 3.0f, 3.0f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 3.0f },
            "student_speeds"_a = std::array<float, 8> { 1.9f, 1.45f, 1.05f, 1.2f, 1.25f, 1.2f, 1.1f, 1.45f },
            "takeover"_a = std::array<float, 5> { 15.0f, 45.0f, 25.0f, 20.0f, 1.0f })
        .def("set_teacher_share", &TrackingGetUpAmpTraining3D::setTeacherShare, "share"_a)
        .def("set_locomotion_assist", &TrackingGetUpAmpTraining3D::setLocomotionAssist, "assist"_a)
        .def("set_balance_feet", &TrackingGetUpAmpTraining3D::setBalanceFeet, "on"_a)
        .def("set_tracking_reward", &TrackingGetUpAmpTraining3D::setTrackingReward, "version"_a)
        .def("set_phase_sync", &TrackingGetUpAmpTraining3D::setPhaseSync, "on"_a)
        .def("set_imitation_loosen", &TrackingGetUpAmpTraining3D::setImitationLoosen, "factor"_a)
        .def("set_facing_gate", &TrackingGetUpAmpTraining3D::setFacingGate, "gate"_a)
        .def("set_arm_clip_reach", &TrackingGetUpAmpTraining3D::setArmClipReach, "reach"_a)
        .def("set_linear_hand", &TrackingGetUpAmpTraining3D::setLinearHand, "on"_a)
        .def("set_upper_hand", &TrackingGetUpAmpTraining3D::setUpperHand, "on"_a)
        .def("set_body_hand", &TrackingGetUpAmpTraining3D::setBodyHand, "on"_a)
        .def("set_idle", &TrackingGetUpAmpTraining3D::setIdle, "after_stand"_a, "on_locomotion"_a, "residual_scale"_a = 1.0f)
        .def("set_idle_imitation", &TrackingGetUpAmpTraining3D::setIdleImitation, "weight"_a)
        .def("set_idle_handoff", &TrackingGetUpAmpTraining3D::setIdleHandoff, "blend_seconds"_a, "blend_reach"_a = -1.0f)
        .def("set_rise_handoff", &TrackingGetUpAmpTraining3D::setRiseHandoff, "head_height"_a)
        .def("set_recovery_natural", &TrackingGetUpAmpTraining3D::setRecoveryNatural, "weight"_a)
        .def("set_lateral_hand_push", &TrackingGetUpAmpTraining3D::setLateralHandPush, "push"_a)
        .def("set_lateral_run_leg_reach", &TrackingGetUpAmpTraining3D::setLateralRunLegReach, "scale"_a)
        .def("set_gait_speeds", &TrackingGetUpAmpTraining3D::setGaitSpeeds, "trot"_a, "run"_a)
        .def("set_back_start_lean", &TrackingGetUpAmpTraining3D::setBackStartLean, "radians"_a)
        .def("set_course_limit", &TrackingGetUpAmpTraining3D::setCourseLimit, "meters"_a)
        .def("set_curve_lean", &TrackingGetUpAmpTraining3D::setCurveLean, "lean"_a)
        .def("set_foot_width", &TrackingGetUpAmpTraining3D::setFootWidth, "slack_meters"_a)
        .def("set_run_turn_brake", &TrackingGetUpAmpTraining3D::setRunTurnBrake, "brake"_a)
        .def("set_run_turn_align", &TrackingGetUpAmpTraining3D::setRunTurnAlign, "align"_a)
        .def("set_turn_settle", &TrackingGetUpAmpTraining3D::setTurnSettle, "settle"_a)
        .def("set_turn_launch", &TrackingGetUpAmpTraining3D::setTurnLaunch, "launch"_a)
        .def("set_cliff_free", &TrackingGetUpAmpTraining3D::setCliffFree, "on"_a)
        .def("set_launch_rise", &TrackingGetUpAmpTraining3D::setLaunchRise, "span_meters"_a)
        .def("set_lateral_start", &TrackingGetUpAmpTraining3D::setLateralStart, "acceleration"_a)
        .def("set_rise_stiffness", &TrackingGetUpAmpTraining3D::setRiseStiffness, "scale"_a)
        .def("set_free_velocity_feedforward", &TrackingGetUpAmpTraining3D::setFreeVelocityFeedforward, "on"_a)
        .def("set_self_teacher_velocity_mode", &TrackingGetUpAmpTraining3D::setSelfTeacherVelocityMode, "mode"_a)
        .def("set_free_damping_scale", &TrackingGetUpAmpTraining3D::setFreeDampingScale, "scale"_a)
        .def("set_velocity_actions", &TrackingGetUpAmpTraining3D::setVelocityActions, "on"_a)
        .def("set_recovery_velocity", &TrackingGetUpAmpTraining3D::setRecoveryVelocity, "on"_a)
        .def("set_teacher_free_drills", &TrackingGetUpAmpTraining3D::setTeacherFreeDrills, "drills"_a)
        .def("set_maneuver_weights", &TrackingGetUpAmpTraining3D::setManeuverWeights, "weights"_a)
        .def("set_turn_settle_lateral", &TrackingGetUpAmpTraining3D::setTurnSettleLateral, "speed"_a)
        .def("set_turn_settle_front", &TrackingGetUpAmpTraining3D::setTurnSettleFront, "speed"_a)
        .def("set_run_turn_time_scale", &TrackingGetUpAmpTraining3D::setRunTurnTimeScale, "scale"_a)
        .def("set_stop_gait_all", &TrackingGetUpAmpTraining3D::setStopGaitAll, "on"_a)
        .def("set_idle_pose_gate", &TrackingGetUpAmpTraining3D::setIdlePoseGate, "gate"_a)
        .def("set_stop_clips_front", &TrackingGetUpAmpTraining3D::setStopClipsFront, "on"_a)
        .def("set_stop_clips_command", &TrackingGetUpAmpTraining3D::setStopClipsCommand, "on"_a)
        .def("set_stop_gate_floor", &TrackingGetUpAmpTraining3D::setStopGateFloor, "floor"_a)
        .def("set_free_stop", &TrackingGetUpAmpTraining3D::setFreeStop, "on"_a)
        .def("set_stop_posture", &TrackingGetUpAmpTraining3D::setStopPosture, "gate"_a)
        .def("set_upper_rigor", &TrackingGetUpAmpTraining3D::setUpperRigor, "factor"_a)
        .def("set_unreal_starts", &TrackingGetUpAmpTraining3D::setUnrealStarts, "on"_a)
        .def("set_unreal_start_scale", &TrackingGetUpAmpTraining3D::setUnrealStartScale, "scale"_a)
        .def("set_live_launch", &TrackingGetUpAmpTraining3D::setLiveLaunch, "on"_a)
        .def("set_launch_step_reward", &TrackingGetUpAmpTraining3D::setLaunchStepReward, "weight"_a)
        .def("set_launch_clip_share", &TrackingGetUpAmpTraining3D::setLaunchClipShare, "share"_a)
        .def("set_launch_hand", &TrackingGetUpAmpTraining3D::setLaunchHand, "level"_a)
        .def("set_launch_in_place_penalty", &TrackingGetUpAmpTraining3D::setLaunchInPlacePenalty, "weight"_a)
        .def("set_launch_calm_arms", &TrackingGetUpAmpTraining3D::setLaunchCalmArms, "on"_a)
        .def("set_spine_velocity_feedforward", &TrackingGetUpAmpTraining3D::setSpineVelocityFeedforward, "on"_a)
        .def("set_base_posture", &TrackingGetUpAmpTraining3D::setBasePosture, "weight"_a)
        .def("set_motor_tone", &TrackingGetUpAmpTraining3D::setMotorTone, "tone"_a)
        .def("set_posture_tail", &TrackingGetUpAmpTraining3D::setPostureTail, "share"_a)
        .def("set_spine_tone", &TrackingGetUpAmpTraining3D::setSpineTone, "tone"_a)
        .def("set_jam_cost", &TrackingGetUpAmpTraining3D::setJamCost, "weight"_a)
        .def("set_foot_slip_cost", &TrackingGetUpAmpTraining3D::setFootSlipCost, "weight"_a)
        .def("set_idle_world_track", &TrackingGetUpAmpTraining3D::setIdleWorldTrack, "on"_a)
        .def("set_motor_gravity", &TrackingGetUpAmpTraining3D::setMotorGravity, "on"_a)
        .def("set_idle_return", &TrackingGetUpAmpTraining3D::setIdleReturn, "weight"_a)
        .def("set_stop_clips_base", &TrackingGetUpAmpTraining3D::setStopClipsBase, "on"_a)
        .def("set_stop_front_variants", &TrackingGetUpAmpTraining3D::setStopFrontVariants, "on"_a, "variant"_a)
        .def("set_stop_front_variant_mask", &TrackingGetUpAmpTraining3D::setStopFrontVariantMask, "mask"_a)
        .def("set_stop_entry", &TrackingGetUpAmpTraining3D::setStopEntry, "blend_seconds"_a, "window_seconds"_a)
        .def("set_free_troca", &TrackingGetUpAmpTraining3D::setFreeTroca, "on"_a)
        .def("set_free_stand_gate", &TrackingGetUpAmpTraining3D::setFreeStandGate, "gate"_a)
        .def("set_free_idle", &TrackingGetUpAmpTraining3D::setFreeIdle, "on"_a)
        .def("set_pose_boost", &TrackingGetUpAmpTraining3D::setPoseBoost, "gate"_a)
        .def("set_free_push", &TrackingGetUpAmpTraining3D::setFreePush, "on"_a)
        .def("set_free_idle_match", &TrackingGetUpAmpTraining3D::setFreeIdleMatch, "weight"_a)
        .def("set_free_step_height", &TrackingGetUpAmpTraining3D::setFreeStepHeight, "meters"_a)
        .def("set_idle_bent_end", &TrackingGetUpAmpTraining3D::setIdleBentEnd, "on"_a)
        .def("set_idle_like_stop", &TrackingGetUpAmpTraining3D::setIdleLikeStop, "on"_a)
        .def("set_idle_wide", &TrackingGetUpAmpTraining3D::setIdleWide, "on"_a)
        .def("set_mirror_cycles", &TrackingGetUpAmpTraining3D::setMirrorCycles, "on"_a)
        .def("set_turn_settle_back", &TrackingGetUpAmpTraining3D::setTurnSettleBack, "speed"_a)
        .def("set_troca_fixed_chest", &TrackingGetUpAmpTraining3D::setTrocaFixedChest, "on"_a)
        .def("set_cliff_parts", &TrackingGetUpAmpTraining3D::setCliffParts, "curve_roll"_a, "foot_cross"_a, "forearm"_a, "hop"_a)
        .def("set_roll_tail", &TrackingGetUpAmpTraining3D::setRollTail, "weight"_a)
        .def("set_course_tail", &TrackingGetUpAmpTraining3D::setCourseTail, "weight"_a)
        .def("set_speed_range", &TrackingGetUpAmpTraining3D::setSpeedRange, "low"_a)
        .def("set_idle_quiet", &TrackingGetUpAmpTraining3D::setIdleQuiet, "weight"_a)
        .def("set_maneuver_acceleration", &TrackingGetUpAmpTraining3D::setManeuverAcceleration, "acceleration"_a)
        .def("set_rise_overshoot", &TrackingGetUpAmpTraining3D::setRiseOvershoot, "weight"_a)
        .def("set_run_launch_ease", &TrackingGetUpAmpTraining3D::setRunLaunchEase, "scale"_a, "ease_seconds"_a, "acceleration"_a)
        .def("set_troca_practice", &TrackingGetUpAmpTraining3D::setTrocaPractice, "practice"_a, "heading_weights"_a)
        .def("set_foot_cross", &TrackingGetUpAmpTraining3D::setFootCross, "slack_meters"_a)
        .def("set_idle_reanchor", &TrackingGetUpAmpTraining3D::setIdleReanchor, "reanchor"_a)
        .def("set_accel_lean", &TrackingGetUpAmpTraining3D::setAccelLean, "radians"_a)
        .def("set_run_lean", &TrackingGetUpAmpTraining3D::setRunLean, "radians"_a, "legs"_a)
        .def("set_run_back_lean", &TrackingGetUpAmpTraining3D::setRunBackLean, "radians"_a, "legs"_a)
        .def("set_get_up_tracking", &TrackingGetUpAmpTraining3D::setGetUpTracking, "on"_a)
        .def("set_sprint", &TrackingGetUpAmpTraining3D::setSprint, "speed"_a, "lean_radians"_a, "legs"_a)
        .def("set_front_lean", &TrackingGetUpAmpTraining3D::setFrontLean, "radians"_a, "start_radians"_a)
        .def("set_balance_steps", &TrackingGetUpAmpTraining3D::setBalanceSteps, "steps"_a, "through_pushes"_a = true)
        .def("set_balance_drill", &TrackingGetUpAmpTraining3D::setBalanceDrill, "fraction"_a, "push_level"_a)
        .def("set_fire", &TrackingGetUpAmpTraining3D::setFire, "episode_share"_a, "hit_newtons"_a, "hold_newtons"_a,
            "hold_seconds"_a, "hold_fraction"_a)
        .def("set_star", &TrackingGetUpAmpTraining3D::setStar, "fraction"_a, "weights"_a, "phases"_a, "seconds"_a)
        .def("set_command", &TrackingGetUpAmpTraining3D::setCommand, "fraction"_a, "phase"_a, "seconds"_a)
        .def("set_final_stop", &TrackingGetUpAmpTraining3D::setFinalStop, "fraction"_a, "earliest"_a, "latest"_a, "deceleration"_a,
            "in_evaluation"_a)
        .def("set_stop_clips", &TrackingGetUpAmpTraining3D::setStopClips, "on"_a)
        .def("set_diagonal_chest", &TrackingGetUpAmpTraining3D::setDiagonalChest, "gate"_a)
        .def("set_course_smoothing", &TrackingGetUpAmpTraining3D::setCourseSmoothing, "seconds"_a)
        .def("set_course_angle_limit", &TrackingGetUpAmpTraining3D::setCourseAngleLimit, "radians"_a)
        .def("set_locomotion_shaping", &TrackingGetUpAmpTraining3D::setLocomotionShaping, "on"_a)
        .def("set_speed_gate", &TrackingGetUpAmpTraining3D::setSpeedGate, "floor"_a)
        .def("set_knee_inward", &TrackingGetUpAmpTraining3D::setKneeInward, "slack"_a)
        .def("set_idle_height", &TrackingGetUpAmpTraining3D::setIdleHeight, "slack"_a)
        .def("set_stop_wait_body", &TrackingGetUpAmpTraining3D::setStopWaitBody, "on"_a)
        .def("set_gait_cycles", &TrackingGetUpAmpTraining3D::setGaitCycles, "on"_a)
        .def("set_push_variety", &TrackingGetUpAmpTraining3D::setPushVariety, "on"_a)
        .def("set_fire_groups", &TrackingGetUpAmpTraining3D::setFireGroups, "bands"_a)
        .def("set_lateral_leg_strict", &TrackingGetUpAmpTraining3D::setLateralLegStrict, "factor"_a)
        .def("set_push_imitation_relief", &TrackingGetUpAmpTraining3D::setPushImitationRelief, "on"_a)
        .def("set_rise_flight", &TrackingGetUpAmpTraining3D::setRiseFlight, "penalty"_a)
        .def("set_slow_limit", &TrackingGetUpAmpTraining3D::setSlowLimit, "fraction"_a)
        .def("set_slow_limits", &TrackingGetUpAmpTraining3D::setSlowLimits, "fractions"_a)
        .def("set_head_clip_reach", &TrackingGetUpAmpTraining3D::setHeadClipReach, "reach"_a)
        .def("set_step_tuning", &TrackingGetUpAmpTraining3D::setStepTuning, "load_limit"_a, "load_weight"_a, "swing_from_body"_a)
        .def("set_avoid_on_idle", &TrackingGetUpAmpTraining3D::setAvoidOnIdle, "on"_a)
        .def("set_get_up_arrival", &TrackingGetUpAmpTraining3D::setGetUpArrival, "bonus"_a, "flight_penalty"_a,
            "rise_balance"_a = 0.0f, "calm_handoff"_a = 0.0f)
        .def("set_idle_threat_reach", &TrackingGetUpAmpTraining3D::setIdleThreatReach, "reach"_a)
        .def_prop_ro("balance_obs_size", &TrackingGetUpAmpTraining3D::balanceObservationSize)
        .def_prop_ro("step_channel_count", &TrackingGetUpAmpTraining3D::stepChannelCount)
        .def("set_run_assist", &TrackingGetUpAmpTraining3D::setRunAssist, "assist"_a)
        .def("set_maneuvers", &TrackingGetUpAmpTraining3D::setManeuvers, "fraction"_a, "evaluation_maneuver"_a = -1)
        .def("set_maneuver_assist", &TrackingGetUpAmpTraining3D::setManeuverAssist, "assist"_a)
        .def("set_drill_assist", [](TrackingGetUpAmpTraining3D& b, const std::vector<float>& levels) { b.setDrillAssist(levels); }, "levels"_a)
        .def("set_teacher_oracle", &TrackingGetUpAmpTraining3D::setTeacherOracle, "mode"_a)
        .def("set_teacher_lead", &TrackingGetUpAmpTraining3D::setTeacherLead, "minimum"_a, "maximum"_a)
        .def("set_teacher_shadow", &TrackingGetUpAmpTraining3D::setTeacherShadow, "shadow"_a)
        .def("set_teacher_still_clip", &TrackingGetUpAmpTraining3D::setTeacherStillClip, "mode"_a)
        .def(
            "set_student_actuator",
            [](TrackingGetUpAmpTraining3D& b, const std::vector<float>& stiffness, const std::vector<float>& damping,
                const std::vector<float>& maximumTorque, bool gravity) {
                b.setStudentActuator(stiffness, damping, maximumTorque, gravity);
            },
            "stiffness"_a, "damping"_a, "maximum_torque"_a, "gravity"_a)
        .def("teacher_drive_stats", [](const TrackingGetUpAmpTraining3D& b) {
            const std::vector<float> stats = b.teacherDriveStats();
            auto out = owned<float>({ stats.size() / 5, 5 });
            std::copy(stats.begin(), stats.end(), out.data());
            return out;
        })
        // O rotulo da ultima decisao de cada mundo: [n, 2*acoes + 3] - o alvo da aluna que daria o torque dela, o alvo dela
        // cru, se vale, se ela estava no volante e se retomou.
        .def("teacher_targets", [](const TrackingGetUpAmpTraining3D& b) {
            auto out = owned<float>({ b.environmentCount(), b.teacherTargetWidth() });
            b.teacherTargets(std::span<float>(out.data(), out.size()));
            return out;
        })
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
        // (07/11, a sonda frente x costas: o torque do motor, o limite dele, o da restricao (limites da junta) e a posicao, por
        // elo e eixo, do ultimo passo.)
        .def("body_joint_telemetry",
            [](const TrackingGetUpAmpTraining3D& b, std::size_t world) {
                const auto& joints = b.body(world).joints;
                auto out = owned<float>({ joints.size(), 12 });
                float* d = out.data();
                for (std::size_t i = 0; i < joints.size(); ++i)
                    for (int k = 0; k < 3; ++k) {
                        d[i * 12 + k] = joints[i].motorTorqueNewtonMeters[k];
                        d[i * 12 + 3 + k] = joints[i].motorTorqueLimitNewtonMeters[k];
                        d[i * 12 + 6 + k] = joints[i].constraintTorqueNewtonMeters[k];
                        d[i * 12 + 9 + k] = joints[i].positionRadians[k];
                    }
                return out;
            },
            "world"_a)
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
        .def("add_style_clips",
            [](TrackingGetUpAmpTraining3D& b, const std::vector<std::string>& clips) { return b.addStyleClips(clips); },
            "clips"_a)
        .def("set_lying_starts", &TrackingGetUpAmpTraining3D::setLyingStarts, "stand_clips"_a, "run_clips"_a)
        .def("set_moving_handoff", &TrackingGetUpAmpTraining3D::setMovingHandoff, "handoff"_a)
        // (07/11) O juiz com o Motion Matching: os bancos, a previsao, os quadros de cada grupo e as consultas por mundo.
        .def("build_motion_matching",
            [](TrackingGetUpAmpTraining3D& b, const std::vector<std::string>& names, const std::vector<std::string>& schemas,
                const std::vector<std::vector<std::string>>& clips, const std::vector<std::array<float, 3>>& biases,
                const std::vector<int>& categories, std::size_t mainGroup) {
                nb::gil_scoped_release release;
                return b.buildMotionMatching(names, schemas, clips, biases, categories, mainGroup);
            },
            "names"_a, "schemas"_a, "clips"_a, "biases"_a, "categories"_a, "main_group"_a)
        .def("set_motion_movement", &TrackingGetUpAmpTraining3D::setMotionMovement, "max_acceleration"_a, "braking_deceleration"_a,
            "ground_friction"_a, "braking_friction"_a, "rotation"_a)
        .def_prop_ro("motion_group_count", &TrackingGetUpAmpTraining3D::motionGroupCount)
        .def_prop_ro("motion_query_width", &TrackingGetUpAmpTraining3D::motionQueryWidth)
        .def_prop_ro("motion_main_group", &TrackingGetUpAmpTraining3D::motionMainGroup)
        .def("motion_query_offset", &TrackingGetUpAmpTraining3D::motionQueryOffset, "group"_a)
        .def("motion_group",
            [](const TrackingGetUpAmpTraining3D& b, std::size_t g) {
                const std::size_t n = b.motionGroupEntries(g), d = b.motionFeatureSize(g), a = b.ampSize();
                auto features = owned<float>({ n, d });
                auto pairs = owned<float>({ n, 2 * a });
                auto addends = owned<float>({ n });
                auto sources = owned<float>({ n });
                auto times = owned<float>({ n });
                b.motionGroupFeatures(g, std::span<float>(features.data(), features.size()));
                b.motionGroupPairs(g, std::span<float>(pairs.data(), pairs.size()));
                b.motionGroupEntryInfo(g, std::span<float>(addends.data(), n), std::span<float>(sources.data(), n),
                    std::span<float>(times.data(), n));
                return nb::make_tuple(features, pairs, addends, sources, times);
            },
            "group"_a)
        .def("motion_queries",
            [](const TrackingGetUpAmpTraining3D& b) {
                auto out = owned<float>({ b.environmentCount(), b.motionQueryWidth() + 2 });
                b.motionQueries(std::span<float>(out.data(), out.size()));
                return out;
            })
        .def("reference_pairs_of",
            [](const TrackingGetUpAmpTraining3D& b, const std::vector<std::string>& clips) {
                std::vector<float> pairs;
                {
                    nb::gil_scoped_release release;
                    pairs = b.referencePairsOf(clips);
                }
                const std::size_t width = 2 * b.ampSize();
                auto out = owned<float>({ pairs.size() / width, width });
                std::copy(pairs.begin(), pairs.end(), out.data());
                return out;
            },
            "clips"_a)
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
                    "parceiro_caiu", "parceiro_s", "tocou_chao", "apoio", "flexao", "obstaculo", "faixa", "cenario",
                    "professora", "retomadas", "rotulados", "ajuda_professora", "desvio_rumo", "fim_motivo", "semelhanca", "estabilidade" };
            })
        .def("finished_episodes",
            [](TrackingGetUpAmpTraining3D& b) {
                const std::vector<TrackingGetUpAmpEpisode3D> episodes = b.takeEpisodes();
                // (04/11 15h: as colunas 35-40, os passos: de recuperacao, de estabilizacao, o equilibrio parado, os empurroes,
                // o pior ponto de captura fora da base (m) e quando quietou de novo depois do ultimo empurrao, s.)
                // (05/11 23h: a 42, o pe cruzado na corrida - footCross.)
                // (08/11: a 43 e a 44, as pernas e a fase da estrela; a 45, se caiu num empurrao ou ate 2 s depois dele.)
                // (08/11 15h20: a 46, o erro da parada no fim - m; < 0 sem parada.)
                constexpr std::size_t width = 47;
                auto out = owned<float>({ episodes.size(), width });
                float* d = out.data();
                for (std::size_t i = 0; i < episodes.size(); ++i) {
                    const TrackingGetUpAmpEpisode3D& e = episodes[i];
                    const float row[width] = { static_cast<float>(e.world), static_cast<float>(e.startKind),
                        static_cast<float>(e.startSide), e.startHeadHeight, e.seconds, e.firstStandSeconds,
                        e.meanTaskReward, e.headHeight, e.pelvisHeight, e.standing ? 1.0f : 0.0f, e.commandSpeed,
                        e.reachedSeconds, e.meanTowardSpeed, e.fellAfterStanding ? 1.0f : 0.0f, e.recomposedSeconds,
                        e.endPoseMatch, e.meanFaceError, e.endSpineMatch, e.handoffSeconds, e.partnerFell ? 1.0f : 0.0f,
                        e.partnerSeconds, e.touchedDown ? 1.0f : 0.0f, e.supported ? 1.0f : 0.0f, e.meanFlex,
                        e.obstructed ? 1.0f : 0.0f, static_cast<float>(e.band), static_cast<float>(e.sceneryPreset),
                        e.teacherShare, static_cast<float>(e.takeovers), static_cast<float>(e.labeled), e.teacherAssist,
                        e.travelError, static_cast<float>(e.endCause), e.meanImitation, e.meanStability,
                        static_cast<float>(e.recoverySteps), static_cast<float>(e.settleSteps), e.balanceDrill ? 1.0f : 0.0f,
                        static_cast<float>(e.pushes), e.worstCaptureOut, e.quietAgainSeconds, e.arrivalQuality, e.footCross,
                        static_cast<float>(e.starLegs), static_cast<float>(e.starPhase), e.fellInPush ? 1.0f : 0.0f, e.stopError };
                    std::copy(row, row + width, d + i * width);
                }
                return out;
            })
        // (08/11) A estrela e o empurrao de um mundo, para as sondas: se e estrela, os 9 pontos (o centro primeiro), o ponto de
        // agora e o seguinte, as pernas, a fase, a chegada pedida (m/s), o pedido (m/s), a velocidade pedida, o rumo da vista e o
        // empurrao de agora (se ha, a forca no plano).
        .def("star_view",
            [](const TrackingGetUpAmpTraining3D& b, std::size_t world) {
                const TrackingGetUpAmpWorldView3D v = b.worldView(world);
                nb::list points;
                for (const Vec3& p : v.starPoints) points.append(nb::make_tuple(p.x, p.y));
                return nb::make_tuple(v.star, points, v.starGoal, v.starNext, v.starLegs, v.starPhase, v.starArrive, v.commandSpeed,
                    nb::make_tuple(v.moveVelocity.x, v.moveVelocity.y), v.faceYaw, v.pushing, nb::make_tuple(v.pushForce.x, v.pushForce.y));
            },
            "world"_a)
        .def("command_text", [](const TrackingGetUpAmpTraining3D& b, std::size_t world) { return b.worldView(world).commandText; },
            "world"_a)
        .def("frame_lines", &TrackingGetUpAmpTraining3D::frameLines, "world"_a)
        .def("reference_lines", &TrackingGetUpAmpTraining3D::referenceLines, "world"_a);
}
