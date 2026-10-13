// A ponte Python do treino de RL (matter_rl_native): o lote de mundos e a
// politica exportada. E a unica parte do projeto que inclui Python. Os
// buffers do lote saem como arrays numpy sem copia (somente leitura; valem
// ate o proximo step), e step() solta o GIL enquanto a fisica roda.
// O Python.h (pelo nanobind) vem antes de qualquer cabecalho do sistema.
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/array.h>

#include "Engine/RL/Environment/TrainingBatch3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Teacher/LocomotionTeacher3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>
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
    // O alvo de agora (o marcador no chao): centro e raio ("T" e o toque na bola).
    if (view.hasTarget)
        out << "A " << view.target.x << ' ' << view.target.y << ' ' << view.target.z << ' ' << view.targetRadius << '\n';
    // Os proximos alvos ja no mapa (N), em ordem.
    for (std::uint32_t i = 0; i < view.upcomingCount; ++i)
        out << "N " << view.upcoming[i].x << ' ' << view.upcoming[i].y << ' ' << view.upcoming[i].z << '\n';
    // Os ja alcancados (R), o mais novo primeiro.
    for (std::uint32_t i = 0; i < view.reachedCount; ++i)
        out << "R " << view.reached[i].x << ' ' << view.reached[i].y << ' ' << view.reached[i].z << '\n';
    // As setas: para onde o cerebro quer ir (W) e a velocidade do centro de massa (V).
    if (view.hasIntent) out << "W " << view.intent.x << ' ' << view.intent.y << ' ' << view.intent.z << '\n';
    out << "V " << view.velocity.x << ' ' << view.velocity.y << ' ' << view.velocity.z << '\n';
    // Para onde a cabeca olha (H) e o rumo mandado olhar (L, rad).
    out << "H " << view.gaze.x << ' ' << view.gaze.y << ' ' << view.gaze.z << '\n';
    if (view.hasFacing) out << "L " << view.facingCommand << '\n';
    return out.str();
}


// As linhas de colisores de uma pose (o formato do dump do quadros).
std::string poseLines(const RagdollProfile3D& profile, const RLBodyPose3D& pose) {
    std::ostringstream out;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        const Vec3 center = pose.positions[i] + pose.orientations[i].rotate(link.collider.localPosition);
        const Quaternion orientation = (pose.orientations[i] * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    return out.str();
}

// Uma referencia de uma lista de clipes (o ultimo em ciclo).
ReferenceMotion3D referenceFromClips(const RLBody3D& body, const std::vector<std::string>& clips) {
    std::vector<ReferenceClip3D> requests;
    for (std::size_t i = 0; i < clips.size(); ++i) requests.push_back({ "", clips[i], i + 1 == clips.size() });
    return buildReferenceMotion3D(body, requests);
}

constexpr float Degrees = 57.29578f;

// O resumo de cada trecho: a emenda, a velocidade, o rumo, a altura da pelve e
// os pes contra o chao (no apoio pelo clipe e no voo).
std::string referenceReport(const std::string& bodyPath, const std::vector<std::string>& clips) {
    const RLBody3D body = loadRLBody3D(bodyPath);
    const ReferenceMotion3D motion = referenceFromClips(body, clips);
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    for (const ReferenceSegment3D& segment : motion.segments) {
        float headingLow = 1e9f, headingHigh = -1e9f, heightLow = 1e9f, heightHigh = -1e9f;
        float stanceHigh = 0.0f, flightLow = 1e9f;
        std::size_t stance = 0, flight = 0, floating = 0;
        for (const ReferenceFrame3D& frame : segment.frames) {
            headingLow = std::min(headingLow, frame.heading);
            headingHigh = std::max(headingHigh, frame.heading);
            heightLow = std::min(heightLow, frame.rootHeight);
            heightHigh = std::max(heightHigh, frame.rootHeight);
            const float lowest = rlBodyLowestFootPoint3D(body, referencePose3D(body, frame, {}, frame.heading));
            if (frame.footContact[0] >= 0.5f || frame.footContact[1] >= 0.5f) {
                ++stance;
                stanceHigh = std::max(stanceHigh, lowest);
                if (lowest > 0.02f) ++floating;
            } else {
                ++flight;
                flightLow = std::min(flightLow, lowest);
            }
        }
        const ReferenceFrame3D& first = segment.frames.front();
        out << "trecho " << segment.id << ": " << segment.frames.size() << " quadros, "
            << (segment.loops ? "em ciclo" : "uma vez") << ", entra no quadro " << segment.entryFrame << " (pulo "
            << segment.entryJumpRms * Degrees << " graus RMS)\n";
        out << "  anda " << segment.meanSpeed << " m/s, para " << (segment.travelHeading - first.heading) * Degrees
            << " graus do rumo da pelve no quadro 0; rumo da pelve de " << headingLow * Degrees << " a "
            << headingHigh * Degrees << " graus\n";
        out << "  altura da pelve de " << heightLow << " a " << heightHigh << " m; quadros no apoio " << stance
            << " (pe mais baixo ate " << stanceHigh << " m acima do chao, " << floating << " acima de 2 cm), no voo "
            << flight << " (pe mais baixo a partir de " << (flight > 0 ? flightLow : 0.0f) << " m)\n";
        out << "  velocidade da pelve no quadro 0: " << first.rootVelocity.x << ' ' << first.rootVelocity.y << ' '
            << first.rootVelocity.z << " m/s\n";
    }
    return out.str();
}

// Quadro a quadro: o apoio pelo clipe e a altura do ponto mais baixo de cada
// pe (a cada step quadros).
std::string referenceFrames(const std::string& bodyPath, const std::vector<std::string>& clips, std::uint32_t step) {
    const RLBody3D body = loadRLBody3D(bodyPath);
    const ReferenceMotion3D motion = referenceFromClips(body, clips);
    const auto lowestOf = [&](const RLBodyPose3D& pose, std::uint32_t foot) {
        const RagdollCapsuleDefinition3D& collider = body.profile.links[foot].collider;
        const Vec3 center = pose.positions[foot] + pose.orientations[foot].rotate(collider.localPosition);
        const Quaternion orientation = pose.orientations[foot] * collider.localOrientation;
        float lowest = 1e9f;
        for (const Vec3& corner : ragdollColliderCorners3D(collider))
            lowest = std::min(lowest, (center + orientation.rotate(corner)).z);
        return lowest;
    };
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    for (const ReferenceSegment3D& segment : motion.segments) {
        out << "trecho " << segment.id << "\n";
        for (std::size_t f = 0; f < segment.frames.size(); f += std::max<std::uint32_t>(1, step)) {
            const ReferenceFrame3D& frame = segment.frames[f];
            const RLBodyPose3D pose = referencePose3D(body, frame, {}, frame.heading);
            out << "  q" << f << " apoio " << frame.footContact[0] << ' ' << frame.footContact[1] << " | pe esq "
                << lowestOf(pose, body.leftFoot) << " dir " << lowestOf(pose, body.rightFoot) << " | pelve "
                << frame.rootHeight << " | v " << frame.rootVelocity.x << ' ' << frame.rootVelocity.y << ' '
                << frame.rootVelocity.z << " | rumo " << frame.heading * Degrees << "\n";
        }
    }
    return out.str();
}

// Um dump do quadros com a referencia tocando no corpo de RL por seconds
// segundos, a pelve andando com a velocidade da referencia.
std::string referenceDump(const std::string& bodyPath, const std::vector<std::string>& clips, float seconds,
    const std::string& scene) {
    const RLBody3D body = loadRLBody3D(bodyPath);
    const ReferenceMotion3D motion = referenceFromClips(body, clips);
    std::ostringstream out;
    out << "S " << scene << '\n';
    ReferenceCursor3D cursor;
    Vec3 position;
    const std::uint32_t total = static_cast<std::uint32_t>(std::lround(seconds / motion.frameSeconds));
    for (std::uint32_t tick = 0; tick <= total; ++tick) {
        const ReferenceFrame3D& frame = referenceFrame3D(motion, cursor);
        if (tick % 4 == 0) {
            char line[64];
            std::snprintf(line, sizeof line, "F %.4f 0\n", static_cast<double>(tick) * motion.frameSeconds);
            out << line << poseLines(body.profile, referencePose3D(body, frame, position, frame.heading));
        }
        const Quaternion world = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, frame.heading);
        position += world.rotate({ frame.rootVelocity.x, frame.rootVelocity.y, 0.0f }) * motion.frameSeconds;
        cursor = advanceReference3D(motion, cursor, 1);
    }
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
                int solverPosition, int solverVelocity, const std::string& posture,
                const std::vector<std::array<float, 4>>& schedule, const std::string& teacherAssets, int facing) {
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
                settings.facing = facing;
                for (const auto& [seconds, forward, lateral, yaw] : schedule)
                    settings.commandSchedule.push_back({ seconds, forward, lateral, yaw });
                nb::gil_scoped_release release;
                // O professor (a locomocao fisica do jogo), com o roteiro da secao
                // "teacher" da tarefa; teacher_assets e a pasta assets do jogo.
                if (!teacherAssets.empty())
                    settings.teacher = std::make_shared<LocomotionTeacher3D>(
                        teacherAssets, loadLocomotionTeacherSettings3D(task));
                new (self) TrainingBatch3D(settings);
            },
            "task"_a, "environments"_a = 256, "per_scene"_a = 4, "threads"_a = 0, "seed"_a = 1,
            "evaluation"_a = false, "evaluation_seconds"_a = 60.0f, "solver_position"_a = -1,
            "solver_velocity"_a = -1, "posture"_a = "", "schedule"_a = std::vector<std::array<float, 4>> {},
            "teacher_assets"_a = "", "facing"_a = -1)
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
        .def(
            "set_command",
            [](TrainingBatch3D& b, std::size_t env, float forward, float lateral, float yaw) {
                b.setCommand(env, { forward, lateral, yaw });
            },
            "env"_a, "forward"_a, "lateral"_a = 0.0f, "yaw"_a = 0.0f)
        .def("clear_command", &TrainingBatch3D::clearCommand, "env"_a)
        .def("root_positions",
            [](const TrainingBatch3D& b) {
                // A pelve de cada mundo (m, no mundo) e a origem dele.
                const std::size_t n = b.environmentCount();
                auto* data = new float[std::max<std::size_t>(1, n) * 6];
                for (std::size_t e = 0; e < n; ++e) {
                    const TrainingEnvironmentView3D view = b.view(e);
                    const Vec3 p = view.state ? view.state->links.front().position : Vec3 {};
                    const float values[6] { p.x, p.y, p.z, view.origin.x, view.origin.y, view.startHeading };
                    std::copy(values, values + 6, data + e * 6);
                }
                nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, { n, 6 }, owner);
            })
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
        .def("teacher_actions",
            [](TrainingBatch3D& b) {
                if (b.teacherActions().empty()) throw std::runtime_error("lote sem professor");
                return view2(b, b.teacherActions(), b.actionSize());
            })
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
                // giro do tronco, giro dos pes no chao, erro da postura, erro da
                // velocidade e do giro, acompanhamento, distancia, desvio lateral,
                // alvos alcancados, segundos ate o primeiro, velocidade media ate
                // eles e o erro da imitacao.
                const auto episodes = b.finishedEpisodes();
                constexpr std::size_t Columns = 26;
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
                    row[15] = e.poseErrorRms;
                    row[16] = e.velocityErrorRms;
                    row[17] = e.yawRateErrorRms;
                    row[18] = e.velocityTracking;
                    row[19] = e.distance;
                    row[20] = e.lateralDrift;
                    row[21] = static_cast<float>(e.targetsReached);
                    row[22] = e.firstTargetSeconds;
                    row[23] = e.meanTargetSpeed;
                    row[24] = e.imitationErrorRms;
                    row[25] = e.facingErrorRms;
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
        "limite", "alvo_perdido" };

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

    m.def("reference_report", &referenceReport, "body"_a, "clips"_a,
        "o resumo da referencia de imitacao nos clipes dados (o ultimo em ciclo)");
    m.def("reference_frames", &referenceFrames, "body"_a, "clips"_a, "step"_a = 4,
        "quadro a quadro: o apoio pelo clipe, os pes contra o chao, a pelve e o rumo");
    m.def("reference_dump", &referenceDump, "body"_a, "clips"_a, "seconds"_a = 3.0f, "scene"_a = "referencia",
        "um dump do quadros com a referencia tocando no corpo de RL");
}
