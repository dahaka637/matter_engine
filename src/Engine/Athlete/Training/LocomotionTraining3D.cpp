#include "Engine/Athlete/Training/LocomotionTraining3D.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Training/LocomotionBatch3D.hpp"
#include "Engine/RL/Commands/CommandPilot3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

namespace MatterEngine {

namespace {

constexpr float Gravity = 9.81f;
constexpr float Pi = 3.14159265359f;
constexpr float TickSeconds = 1.0f / 120.0f;
constexpr int TicksPerDecision = LocomotionContract3D::TicksPerDecision;
// Um pouso conta depois de o pe ficar no ar ao menos isto (s); o pe apoiado
// escorrega depois de 0,05 s no chao (sem o impacto do pouso).
constexpr float TouchdownAirSeconds = 0.10f;
constexpr int SlipAfterTicks = 6;
// A meta de pouso da referencia vale para o pe no ar com o pouso ate isto a
// frente (s).
constexpr float TouchdownGoalHorizon = 0.5f;

float wrap(float angle) { return std::remainder(angle, 2.0f * Pi); }

int findLink(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<int>(i);
    return -1;
}

std::size_t channelIndex(const RLVectorLayout3D& layout, const std::string& name) {
    for (std::size_t i = 0; i < layout.channels.size(); ++i)
        if (layout.channels[i].name == name) return i;
    throw std::runtime_error("treino da locomocao: canal sumiu do contrato: " + name);
}

// O rumo no plano (o referencial da rede).
struct Frame {
    float c = 1.0f;
    float s = 0.0f;
    [[nodiscard]] Vec3 in(Vec3 v) const { return { c * v.x + s * v.y, -s * v.x + c * v.y, v.z }; }
    [[nodiscard]] Vec3 out(Vec3 v) const { return { c * v.x - s * v.y, s * v.x + c * v.y, v.z }; }
};

Frame frameOf(float yaw) { return { std::cos(yaw), std::sin(yaw) }; }

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

AthleteRuntimeSettings3D runtimeFor(const LocomotionTask3D& task) {
    AthleteRuntimeSettings3D runtime;
    // O fisico total (o atleta comandado so roda no fisico), a referencia a
    // sombra (o estilo e as metas de contato) e a ajuda do episodio.
    runtime.defaultLocomotion = false;
    runtime.physicalReference = true;
    runtime.e5StandingAssistPercent = task.standingAssistPercent;
    runtime.e5WalkingAssistPercent = task.walkingAssistPercent;
    runtime.physicalFeetAssistPercent = task.feetAssistPercent;
    runtime.getUpAssistPercent = task.getUpAssistPercent;
    // A virada brusca: o episodio liga ou nao (o dropout, no recomeco).
    runtime.dynamicTurnMoveSettings.amount = task.turnMove.amount;
    return runtime;
}

} // namespace

const char* locomotionEndName3D(LocomotionEnd3D end) {
    switch (end) {
    case LocomotionEnd3D::None: return "nenhum";
    case LocomotionEnd3D::Fall: return "queda";
    case LocomotionEnd3D::Numeric: return "numerico";
    case LocomotionEnd3D::TimeLimit: return "tempo";
    case LocomotionEnd3D::Boundary: return "limite";
    case LocomotionEnd3D::Failure: return "falha";
    }
    return "?";
}

const std::vector<std::string>& locomotionEpisodeColumns3D() {
    static const std::vector<std::string> columns = [] {
        std::vector<std::string> c { "mundo", "exercicio", "razao", "decisoes", "segundos", "retorno", "erro_velocidade",
            "erro_rumo", "erro_estilo", "contato_por_pouso", "pousos_por_segundo", "escorregar", "potencia", "ajuda",
            "postura", "residuo", "trancos", "pedido_medio", "velocidade_media", "distancia", "alvos_por_minuto",
            "deriva_espera", "afastamento_curva", "erro_vel_janela", "erro_vel_cruzeiro" };
        for (const char* term : LocomotionRewardTermNames3D) c.push_back(std::string("nota_") + term);
        return c;
    }();
    return columns;
}

std::vector<float> locomotionEpisodeRow3D(const LocomotionEpisode3D& e) {
    std::vector<float> row { static_cast<float>(e.environment), static_cast<float>(e.drill),
        static_cast<float>(static_cast<int>(e.reason)), static_cast<float>(e.steps), e.seconds, e.episodeReturn,
        e.velocityError, e.facingError, e.styleError, e.contactPerTouchdown, e.touchdownsPerSecond, e.slip, e.power,
        e.assist, e.posture, e.residual, e.jerk, e.meanRequested, e.meanSpeed, e.distance, e.targetsPerMinute,
        e.holdDrift, e.arcError, e.velocityErrorWindow, e.velocityErrorCruise };
    row.insert(row.end(), e.terms.begin(), e.terms.end());
    return row;
}

struct LocomotionTraining3D::Impl {
    LocomotionTrainingSettings3D settings;
    LocomotionTask3D task;
    std::shared_ptr<const AthleteBundle3D> bundle;
    std::unique_ptr<LocomotionContract3D> contract;
    std::unique_ptr<LocomotionBatch3D> batch;
    LocomotionAuthority3D authority;
    // A virada brusca nos episodios novos (setTurnMoveMode).
    int turnMoveMode = 0;
    RLVectorLayout3D critic;
    std::size_t observationSize = 0;
    std::size_t criticSize = 0;
    std::size_t actionSize = 0;
    std::size_t drillCount = 0;
    int fixedDrill = -1;
    std::vector<float> drillWeights;
    // O teto de cada canal com a autoridade inteira (o maior dos dois lados):
    // o residuo da rede e medido contra ele.
    std::vector<float> capMax;
    // Os links e os canais da entrada que a nota le.
    int chest = -1;
    int head = -1;
    std::array<int, 2> feet { -1, -1 };
    std::array<int, 2> toes { -1, -1 };
    std::size_t referenceValid = 0;
    std::array<std::size_t, 6> styleError {};
    struct FootGoalChannels {
        std::size_t valid = 0, x = 0, y = 0, when = 0;
    };
    std::array<FootGoalChannels, 2> footGoal {};
    float mass = 1.0f;

    struct World {
        std::mt19937_64 rng;
        std::uint32_t drill = 0;
        PilotState3D pilot;
        AthleteCommand3D command;
        BodyForecast3D forecast;
        // A velocidade viavel (o pedido no limite de aceleracao do fisico).
        Vec3 feasible;
        Vec3 comVelocity;
        bool comValid = false;
        // O diagnostico do erro de velocidade (so medida, fora da nota): a
        // velocidade e a viavel passadas pelo mesmo filtro de ~um passo, e o
        // erro so quando a viavel ja chegou ao pedido (o cruzeiro).
        Vec3 comSmooth, feasibleSmooth;
        double windowError2 = 0.0, cruiseError2 = 0.0;
        int windowTicks = 0, cruiseTicks = 0;
        Vec3 measuredAcceleration;
        std::vector<float> lastAction;
        std::array<int, 2> contactTicks {};
        std::array<float, 2> airSeconds {};
        std::array<bool, 2> goalValid {};
        std::array<Vec3, 2> goal {};
        Vec3 start;
        std::uint32_t steps = 0;
        float seconds = 0.0f;
        double episodeReturn = 0.0;
        double velocityError2 = 0.0, facingError2 = 0.0, styleError2 = 0.0;
        int styleSamples = 0;
        double contactScore = 0.0;
        int contactGoals = 0;
        int touchdowns = 0;
        double slip = 0.0, power = 0.0, assist = 0.0, posture = 0.0, residual = 0.0, jerk = 0.0;
        double requested = 0.0, speed = 0.0;
        std::array<double, LocomotionRewardTermCount3D> terms {};
        bool hasEpisode = false;
        LocomotionEpisode3D episode;
    };
    std::vector<World> worlds;
    std::vector<std::uint32_t> worldDrills;
    std::vector<float> observations;
    std::vector<float> criticObservations;
    std::vector<float> terminalCritic;
    std::vector<float> rewardTerms;
    std::vector<float> rewards;
    std::vector<std::uint8_t> terminated;
    std::vector<std::uint8_t> truncated;
    std::vector<LocomotionEpisode3D> finished;
    double lastStepMilliseconds = 0.0;

    [[nodiscard]] float episodeSeconds() const {
        return settings.evaluation ? settings.evaluationSeconds : task.episodeSeconds;
    }

    void computeCaps() {
        const LocomotionAuthority3D full = LocomotionAuthority3D::uniform(1.0f, task.caps);
        const std::vector<float> up = contract->physical(std::vector<float>(actionSize, 1.0f), full);
        const std::vector<float> down = contract->physical(std::vector<float>(actionSize, -1.0f), full);
        capMax.assign(actionSize, 0.0f);
        for (std::size_t i = 0; i < actionSize; ++i) capMax[i] = std::max(std::abs(up[i]), std::abs(down[i]));
        authority.caps = task.caps;
    }

    std::uint32_t pickDrill(World& world) {
        if (fixedDrill >= 0) return static_cast<std::uint32_t>(fixedDrill);
        float total = 0.0f;
        for (float w : drillWeights) total += std::max(0.0f, w);
        if (!(total > 0.0f)) return 0;
        float pick = std::uniform_real_distribution<float>(0.0f, total)(world.rng);
        for (std::size_t d = 0; d < drillWeights.size(); ++d) {
            pick -= std::max(0.0f, drillWeights[d]);
            if (pick <= 0.0f) return static_cast<std::uint32_t>(d);
        }
        return static_cast<std::uint32_t>(drillWeights.size() - 1);
    }

    [[nodiscard]] Vec3 centerOfMassVelocity(const RagdollState3D& body) const {
        const RagdollProfile3D& profile = bundle->profile();
        Vec3 sum;
        float weight = 0.0f;
        for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
            sum += body.links[i].linearVelocity * profile.links[i].massFraction;
            weight += profile.links[i].massFraction;
        }
        return weight > 0.0f ? sum * (1.0f / weight) : Vec3 {};
    }

    [[nodiscard]] bool touching(const RagdollState3D& body, std::size_t side) const {
        for (const RagdollInteraction3D& contact : body.interactions) {
            const int link = static_cast<int>(contact.linkIndex);
            if ((link == feet[side] || link == toes[side]) && contact.normalWorld.z > 0.5f) return true;
        }
        return false;
    }

    // O comando do piloto pelo controle do jogo: a direcao no referencial da
    // camera e a velocidade pedida, na tabela do jogo para aquela direcao.
    AthleteCommand3D pilotCommand(std::size_t index, World& world, float dt) {
        LocomotionWorld3D& scene = batch->world(index);
        const RagdollState3D& body = scene.body();
        PilotView3D view;
        if (!body.links.empty()) view.position = body.links.front().position;
        view.velocity = world.comVelocity;
        const ControlInput3D input = CommandPilot3D::update(task.drills[world.drill].pilot, world.pilot, view, dt, world.rng);
        const Frame camera = frameOf(input.cameraHeading);
        Vec3 direction = camera.out({ input.stickForward, input.stickLeft, 0.0f });
        const float magnitude = std::hypot(direction.x, direction.y);
        AthleteStickInput3D stick;
        stick.cameraYawRadians = input.cameraHeading;
        stick.lookPitchRadians = -0.2f;
        const AthleteRuntimeSettings3D& runtime = scene.runtimeSettings();
        if (magnitude > 1.0e-4f) {
            direction = direction * (1.0f / magnitude);
            stick.moveWorld = direction;
            const auto tableSpeed = [&]() {
                const Vec3 table = athleteStickCommand3D(stick, runtime, bundle->gaitSpeeds(), 1.0e9f)
                                       .intent.requestedVelocityWorld;
                return std::hypot(table.x, table.y);
            };
            const float wanted = std::max(0.0f, input.speed) * std::min(1.0f, magnitude);
            float table = tableSpeed();
            // Acima do trote da tabela, o sprint (o jogador segurando a tecla).
            if (wanted > table + 1.0e-3f) {
                stick.sprint = true;
                table = tableSpeed();
            }
            stick.moveWorld = direction * (table > 1.0e-3f ? std::min(1.0f, wanted / table) : 0.0f);
        }
        return athleteStickCommand3D(stick, runtime, bundle->gaitSpeeds(), scene.telemetry().terrainSpeedLimit);
    }

    void startEpisode(std::size_t index) {
        World& world = worlds[index];
        LocomotionWorld3D& scene = batch->world(index);
        world.drill = pickDrill(world);
        worldDrills[index] = world.drill;
        scene.reset(AthleteStartMode3D::Physical);
        LocomotionContract3D::clear(scene.athlete());
        // A virada brusca do episodio (so com a tarefa ligada: desligada, nem
        // o sorteio acontece e o mundo e o de antes, bit a bit).
        if (task.turnMove.enabled) {
            const bool on = turnMoveMode == 1 ? false
                : turnMoveMode == 2 || settings.evaluation ? true
                : std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) >= task.turnMove.dropout;
            scene.runtimeSettings().dynamicTurnMove = on;
            scene.runtimeSettings().dynamicTurnMoveSettings.amount = task.turnMove.amount;
        } else {
            // (Uma tarefa trocada andando, de ligada para desligada: o mundo
            // volta ao classico no recomeco.)
            scene.runtimeSettings().dynamicTurnMove = false;
        }
        const RagdollState3D& body = scene.body();
        const Vec3 pelvis = body.links.empty() ? Vec3 {} : body.links.front().position;
        world.start = planar(pelvis);
        // (O corpo nasce olhando para +x; a camera perto disso. Na avaliacao
        // tambem, pelo sorteio do mundo - reproduzivel pela semente: com a
        // camera exata em 0, os mundos parados eram copias bit a bit de um so.)
        const float camera = std::uniform_real_distribution<float>(-0.25f, 0.25f)(world.rng);
        world.pilot = {};
        CommandPilot3D::start(task.drills[world.drill].pilot, world.pilot, world.start, camera, world.rng);
        world.command = {};
        world.command.facingYawRadians = camera;
        world.command.intent.requestedFacingYawRadians = camera;
        world.command.lookPitchRadians = -0.2f;
        world.feasible = {};
        world.comVelocity = {};
        world.comValid = false;
        world.comSmooth = world.feasibleSmooth = {};
        world.windowError2 = world.cruiseError2 = 0.0;
        world.windowTicks = world.cruiseTicks = 0;
        world.measuredAcceleration = {};
        world.lastAction.assign(actionSize, 0.0f);
        world.contactTicks = {};
        world.airSeconds = {};
        world.goalValid = {};
        world.steps = 0;
        world.seconds = 0.0f;
        world.episodeReturn = 0.0;
        world.velocityError2 = world.facingError2 = world.styleError2 = 0.0;
        world.styleSamples = 0;
        world.contactScore = 0.0;
        world.contactGoals = world.touchdowns = 0;
        world.slip = world.power = world.assist = world.posture = world.residual = world.jerk = 0.0;
        world.requested = world.speed = 0.0;
        world.terms = {};
        for (std::size_t side = 0; side < 2; ++side) world.contactTicks[side] = touching(body, side) ? SlipAfterTicks : 0;
    }

    // A entrada e o critico de um mundo agora; volta quantos numeros nao
    // finitos a entrada teve.
    std::size_t observe(std::size_t index, std::span<float> obs, std::span<float> criticRow) {
        World& world = worlds[index];
        LocomotionWorld3D& scene = batch->world(index);
        const AthleteState3D& athlete = scene.athlete();
        const RagdollState3D& body = scene.body();
        const AthleteControlView3D view = athleteControlView3D(athlete);
        const LocomotionObservation3D written =
            contract->observe(body, world.command, view, athlete.policyHistory, world.forecast, obs);
        std::copy(obs.begin(), obs.end(), criticRow.begin());
        std::size_t c = observationSize;
        for (std::size_t d = 0; d < drillCount; ++d) criticRow[c++] = d == world.drill ? 1.0f : 0.0f;
        const float yaw = body.links.empty() ? 0.0f : locomotionFrameYaw3D(bundle->profile(), body.links.front().orientation);
        const Frame frame = frameOf(yaw);
        const Vec3 pelvis = body.links.empty() ? Vec3 {} : body.links.front().position;
        const Vec3 target = world.pilot.hasTarget ? frame.in(planar(world.pilot.target - pelvis)) : Vec3 {};
        criticRow[c++] = std::clamp(target.x, -20.0f, 20.0f);
        criticRow[c++] = std::clamp(target.y, -20.0f, 20.0f);
        criticRow[c++] = world.pilot.hasTarget && world.pilot.stopAtTarget ? 1.0f : 0.0f;
        criticRow[c++] = world.pilot.holdLeft;
        const Vec3 feasible = frame.in(world.feasible);
        criticRow[c++] = feasible.x;
        criticRow[c++] = feasible.y;
        criticRow[c++] = world.seconds / std::max(1.0f, episodeSeconds());
        for (float& v : criticRow)
            if (!std::isfinite(v)) v = 0.0f;
        return written.nonFinite;
    }

    // Uma decisao de um mundo: a acao, o piloto, os 4 ticks (a rampa, o passo,
    // as medidas), a entrada, a nota e o fim.
    void stepWorld(std::size_t index, std::span<const float> action) {
        World& world = worlds[index];
        LocomotionWorld3D& scene = batch->world(index);
        AthleteState3D& athlete = scene.athlete();
        const RagdollProfile3D& profile = bundle->profile();
        const float decisionSeconds = TickSeconds * TicksPerDecision;
        world.hasEpisode = false;
        std::span<float> obs(observations.data() + index * observationSize, observationSize);
        std::span<float> criticRow(criticObservations.data() + index * criticSize, criticSize);
        std::span<float> terms(rewardTerms.data() + index * LocomotionRewardTermCount3D, LocomotionRewardTermCount3D);
        terminated[index] = truncated[index] = 0;

        LocomotionStepMeasure3D measure;
        double jerk = 0.0;
        for (std::size_t i = 0; i < actionSize; ++i) {
            const float a = std::isfinite(action[i]) ? std::clamp(action[i], -1.0f, 1.0f) : 0.0f;
            jerk += (a - world.lastAction[i]) * (a - world.lastAction[i]);
            world.lastAction[i] = a;
        }
        measure.jerk = static_cast<float>(jerk / std::max<std::size_t>(1, actionSize));
        contract->decide(athlete, action);
        world.command = pilotCommand(index, world, decisionSeconds);
        const AthleteControlView3D before = athleteControlView3D(athlete);
        const float acceleration = std::max(0.5f, before.navigation.acceleration);
        const float deceleration = std::max(0.5f, before.navigation.deceleration);

        double velocityError2 = 0.0, facingError2 = 0.0, slip = 0.0, power = 0.0, posture = 0.0, assist = 0.0;
        const float weight = Gravity * std::max(1.0f, bundle->profile().totalMassKg);
        int slipSamples = 0, ticks = 0;
        bool fell = false, numeric = false;
        const Vec3 asked = planar(world.command.intent.requestedVelocityWorld);
        for (int tick = 0; tick < TicksPerDecision; ++tick) {
            contract->follow(athlete, authority, TickSeconds);
            scene.step(world.command);
            const RagdollState3D& body = scene.body();
            if (body.links.empty() || !finite(body.links.front().position)) {
                numeric = true;
                break;
            }
            ++ticks;
            // A velocidade viavel: o pedido no limite de aceleracao do fisico.
            {
                Vec3 change = asked - world.feasible;
                const float limit = (change.x * world.feasible.x + change.y * world.feasible.y < 0.0f
                    ? deceleration : acceleration) * TickSeconds;
                const float size = std::hypot(change.x, change.y);
                if (size > limit) change = change * (limit / size);
                world.feasible += change;
            }
            const Vec3 com = planar(centerOfMassVelocity(body));
            if (world.comValid) {
                const Vec3 measured = (com - world.comVelocity) * (1.0f / TickSeconds);
                world.measuredAcceleration += (measured - world.measuredAcceleration)
                    * (1.0f - std::exp(-TickSeconds / 0.25f));
            }
            world.comVelocity = com;
            world.comValid = true;
            const Vec3 error = com - world.feasible;
            velocityError2 += error.x * error.x + error.y * error.y;
            {
                const float blend = 1.0f - std::exp(-TickSeconds / 0.25f);
                world.comSmooth += (com - world.comSmooth) * blend;
                world.feasibleSmooth += (world.feasible - world.feasibleSmooth) * blend;
                const Vec3 smooth = world.comSmooth - world.feasibleSmooth;
                world.windowError2 += smooth.x * smooth.x + smooth.y * smooth.y;
                ++world.windowTicks;
                const Vec3 gap = asked - world.feasible;
                if (std::hypot(gap.x, gap.y) < 0.05f) {
                    world.cruiseError2 += error.x * error.x + error.y * error.y;
                    ++world.cruiseTicks;
                }
            }
            const float yaw = locomotionFrameYaw3D(profile, body.links.front().orientation);
            const Frame frame = frameOf(yaw);
            if (chest >= 0 && static_cast<std::size_t>(chest) < body.links.size()) {
                const Quaternion r = (body.links[static_cast<std::size_t>(chest)].orientation
                    * profile.links[static_cast<std::size_t>(chest)].modelOrientation.conjugate()).normalized();
                const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
                const float facing = wrap(std::atan2(forward.y, forward.x) - world.command.facingYawRadians);
                facingError2 += facing * facing;
                LocomotionPostureInput3D input;
                input.chestUp = frame.in(r.rotate({ 0.0f, 0.0f, 1.0f }));
                input.measuredAcceleration = frame.in(world.measuredAcceleration);
                Vec3 commanded = (asked - com) * (1.0f / 0.25f);
                const float size = std::hypot(commanded.x, commanded.y);
                commanded = size < 0.3f ? Vec3 {} : commanded * (std::min(size, 8.0f) * 0.5f / size);
                input.commandAcceleration = frame.in(commanded);
                input.speed = std::hypot(com.x, com.y);
                posture += locomotionPostureExcess3D(input);
            }
            for (std::size_t side = 0; side < 2; ++side) {
                if (feet[side] < 0 || static_cast<std::size_t>(feet[side]) >= body.links.size()) continue;
                const Vec3 foot = body.links[static_cast<std::size_t>(feet[side])].position;
                if (touching(body, side)) {
                    if (world.contactTicks[side] == 0 && world.airSeconds[side] >= TouchdownAirSeconds) {
                        ++world.touchdowns;
                        if (world.goalValid[side]) {
                            const float dx = foot.x - world.goal[side].x, dy = foot.y - world.goal[side].y;
                            const float sigma = task.reward.contactSigma;
                            measure.contactScore += std::exp(-(dx * dx + dy * dy) / (sigma * sigma));
                            ++world.contactGoals;
                            world.goalValid[side] = false;
                        }
                    }
                    ++world.contactTicks[side];
                    world.airSeconds[side] = 0.0f;
                    if (world.contactTicks[side] >= SlipAfterTicks) {
                        const Vec3 v = body.links[static_cast<std::size_t>(feet[side])].linearVelocity;
                        slip += std::hypot(v.x, v.y);
                        ++slipSamples;
                    }
                } else {
                    world.contactTicks[side] = 0;
                    world.airSeconds[side] += TickSeconds;
                }
            }
            double watts = 0.0;
            for (const RagdollJointState3D& joint : body.joints)
                for (std::size_t axis = 0; axis < 3; ++axis)
                    watts += std::abs(joint.motorTorqueNewtonMeters[axis] * joint.velocityRadiansPerSecond[axis]);
            power += watts / mass;
            // A ajuda auxiliar neste tick (todas as forcas de fora da regua, fracao do peso).
            assist += athlete.externalAssist.sliderNewtons() / weight;
            if (body.links.front().position.z - athlete.groundHeight < task.fallPelvisHeight) {
                fell = true;
                break;
            }
        }
        if (!numeric && !fell && scene.telemetry().fallen) fell = true;
        const float tickCount = static_cast<float>(std::max(1, ticks));
        measure.velocityError = static_cast<float>(std::sqrt(velocityError2 / tickCount));
        measure.facingError = static_cast<float>(std::sqrt(facingError2 / tickCount));
        measure.slip = slipSamples > 0 ? static_cast<float>(slip / slipSamples) : 0.0f;
        measure.power = static_cast<float>(power / tickCount);
        measure.posture = static_cast<float>(posture / tickCount);
        measure.assist = static_cast<float>(assist / tickCount);
        measure.fell = fell;

        // A entrada de agora (e o que a nota le dela: o estilo e as metas de
        // pouso da referencia).
        const std::size_t nonFinite = observe(index, obs, criticRow);
        if (nonFinite > 0) numeric = true;
        if (obs[referenceValid] > 0.5f) {
            double sum = 0.0;
            for (std::size_t part = 0; part < styleError.size(); ++part)
                for (std::size_t axis = 0; axis < 3; ++axis)
                    sum += obs[styleError[part] + axis] * obs[styleError[part] + axis];
            measure.styleValid = true;
            measure.styleError = static_cast<float>(std::sqrt(sum / static_cast<double>(styleError.size())));
        }
        {
            const RagdollState3D& body = scene.body();
            if (!body.links.empty()) {
                const Frame frame = frameOf(locomotionFrameYaw3D(profile, body.links.front().orientation));
                const Vec3 ground { body.links.front().position.x, body.links.front().position.y, athlete.groundHeight };
                for (std::size_t side = 0; side < 2; ++side) {
                    const FootGoalChannels& g = footGoal[side];
                    const bool valid = obs[g.valid] > 0.5f && obs[g.when] >= 0.0f && obs[g.when] <= TouchdownGoalHorizon
                        && world.contactTicks[side] == 0;
                    world.goalValid[side] = valid;
                    if (valid) world.goal[side] = ground + frame.out({ obs[g.x], obs[g.y], 0.0f });
                }
            }
        }
        {
            double residual = 0.0;
            std::size_t counted = 0;
            for (std::size_t i = 0; i < actionSize && i < athlete.policyApplied.size(); ++i) {
                if (!(capMax[i] > 0.0f)) continue;
                const double r = athlete.policyApplied[i] / capMax[i];
                residual += r * r;
                ++counted;
            }
            measure.residual = counted > 0 ? static_cast<float>(residual / static_cast<double>(counted)) : 0.0f;
        }
        const std::array<float, LocomotionRewardTermCount3D> values =
            locomotionRewardTerms3D(measure, task.reward, decisionSeconds);
        std::copy(values.begin(), values.end(), terms.begin());
        rewards[index] = numeric ? 0.0f : locomotionReward3D(values);

        // O episodio.
        ++world.steps;
        world.seconds += decisionSeconds;
        world.episodeReturn += rewards[index];
        world.velocityError2 += measure.velocityError * measure.velocityError;
        world.facingError2 += measure.facingError * measure.facingError;
        if (measure.styleValid) {
            world.styleError2 += measure.styleError * measure.styleError;
            ++world.styleSamples;
        }
        world.contactScore += measure.contactScore;
        world.slip += measure.slip;
        world.power += measure.power;
        world.assist += measure.assist;
        world.posture += measure.posture;
        world.residual += measure.residual;
        world.jerk += measure.jerk;
        world.requested += std::hypot(asked.x, asked.y);
        world.speed += std::hypot(world.comVelocity.x, world.comVelocity.y);
        for (std::size_t t = 0; t < values.size(); ++t) world.terms[t] += values[t];

        LocomotionEnd3D end = LocomotionEnd3D::None;
        const RagdollState3D& body = scene.body();
        const Vec3 pelvis = body.links.empty() ? Vec3 {} : body.links.front().position;
        if (numeric) end = LocomotionEnd3D::Numeric;
        else if (fell) end = LocomotionEnd3D::Fall;
        else if (world.seconds >= episodeSeconds() - 1.0e-4f) end = LocomotionEnd3D::TimeLimit;
        else if (std::hypot(pelvis.x, pelvis.y) > task.boundaryMeters) end = LocomotionEnd3D::Boundary;
        if (end == LocomotionEnd3D::None) return;
        if (end == LocomotionEnd3D::Fall || end == LocomotionEnd3D::Numeric) terminated[index] = 1;
        else truncated[index] = 1;
        finishEpisode(index, end);
        std::copy(criticRow.begin(), criticRow.end(), terminalCritic.begin() + static_cast<std::ptrdiff_t>(index * criticSize));
        startEpisode(index);
        (void)observe(index, obs, criticRow);
    }

    void finishEpisode(std::size_t index, LocomotionEnd3D reason) {
        World& world = worlds[index];
        const double steps = std::max<std::uint32_t>(1, world.steps);
        LocomotionEpisode3D& e = world.episode;
        e = {};
        e.environment = static_cast<std::uint32_t>(index);
        e.drill = world.drill;
        e.reason = reason;
        e.steps = world.steps;
        e.seconds = world.seconds;
        e.episodeReturn = static_cast<float>(world.episodeReturn);
        e.velocityError = static_cast<float>(std::sqrt(world.velocityError2 / steps));
        e.facingError = static_cast<float>(std::sqrt(world.facingError2 / steps));
        e.styleError = world.styleSamples > 0 ? static_cast<float>(std::sqrt(world.styleError2 / world.styleSamples)) : -1.0f;
        e.contactPerTouchdown = world.contactGoals > 0 ? static_cast<float>(world.contactScore / world.contactGoals) : -1.0f;
        e.touchdownsPerSecond = world.seconds > 0.0f ? static_cast<float>(world.touchdowns) / world.seconds : 0.0f;
        e.slip = static_cast<float>(world.slip / steps);
        e.power = static_cast<float>(world.power / steps);
        e.assist = static_cast<float>(world.assist / steps);
        e.posture = static_cast<float>(world.posture / steps);
        e.residual = static_cast<float>(world.residual / steps);
        e.jerk = static_cast<float>(world.jerk / steps);
        e.meanRequested = static_cast<float>(world.requested / steps);
        e.meanSpeed = static_cast<float>(world.speed / steps);
        const RagdollState3D& body = batch->world(index).body();
        if (!body.links.empty()) {
            const Vec3 moved = planar(body.links.front().position) - world.start;
            e.distance = std::hypot(moved.x, moved.y);
        }
        e.targetsPerMinute = world.seconds > 0.0f ? static_cast<float>(world.pilot.reached) * 60.0f / world.seconds : 0.0f;
        e.holdDrift = world.pilot.holdDecisions > 0
            ? world.pilot.holdDistanceSum / static_cast<float>(world.pilot.holdDecisions) : -1.0f;
        e.velocityErrorWindow = world.windowTicks > 0
            ? static_cast<float>(std::sqrt(world.windowError2 / world.windowTicks)) : -1.0f;
        e.velocityErrorCruise = world.cruiseTicks > 0
            ? static_cast<float>(std::sqrt(world.cruiseError2 / world.cruiseTicks)) : -1.0f;
        e.arcError = world.pilot.arcDecisions > 0 ? world.pilot.arcErrorSum / static_cast<float>(world.pilot.arcDecisions)
                                                  : -1.0f;
        for (std::size_t t = 0; t < world.terms.size(); ++t) e.terms[t] = static_cast<float>(world.terms[t]);
        world.hasEpisode = true;
    }
};

LocomotionTraining3D::LocomotionTraining3D(const LocomotionTrainingSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.task = loadLocomotionTask3D(settings.taskPath);
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    const RagdollProfile3D& profile = impl.bundle->profile();
    impl.contract = std::make_unique<LocomotionContract3D>(profile);
    impl.observationSize = impl.contract->observationSize();
    impl.actionSize = impl.contract->actionSize();
    impl.drillCount = impl.task.drills.size();
    impl.mass = std::max(1.0f, profile.totalMassKg);
    if (!settings.drill.empty()) {
        for (std::size_t d = 0; d < impl.task.drills.size(); ++d)
            if (impl.task.drills[d].id == settings.drill) impl.fixedDrill = static_cast<int>(d);
        if (impl.fixedDrill < 0) throw std::runtime_error("treino da locomocao: exercicio desconhecido: " + settings.drill);
    }
    impl.drillWeights.clear();
    for (const LocomotionDrill3D& drill : impl.task.drills) impl.drillWeights.push_back(drill.weight);

    impl.chest = findLink(profile, "Chest");
    impl.head = findLink(profile, "Head");
    impl.feet = { findLink(profile, "LeftFoot"), findLink(profile, "RightFoot") };
    impl.toes = { findLink(profile, "LeftToes"), findLink(profile, "RightToes") };
    if (impl.toes[0] < 0) impl.toes = { findLink(profile, "LeftToe"), findLink(profile, "RightToe") };
    const RLVectorLayout3D& layout = impl.contract->observationLayout();
    impl.referenceValid = channelIndex(layout, "ref.valida");
    const char* const parts[6] = { "pe_esq", "pe_dir", "mao_esq", "mao_dir", "peito", "cabeca" };
    for (std::size_t p = 0; p < 6; ++p)
        impl.styleError[p] = channelIndex(layout, std::string("ref.estilo_erro.") + parts[p] + ".x");
    for (std::size_t side = 0; side < 2; ++side) {
        const std::string foot = std::string("ref.pe_") + (side == 0 ? "esq" : "dir");
        impl.footGoal[side] = { channelIndex(layout, foot + ".pouso_valido"), channelIndex(layout, foot + ".pouso.x"),
            channelIndex(layout, foot + ".pouso.y"), channelIndex(layout, foot + ".pousa_em") };
    }

    // O critico: a entrada e o que so o treino sabe (o exercicio, o alvo do
    // piloto, a velocidade viavel, o tempo).
    impl.critic = layout;
    impl.critic.id = "locomocao.critico";
    for (const LocomotionDrill3D& drill : impl.task.drills) impl.critic.channels.push_back({ "drill." + drill.id, "1" });
    for (const char* name : { "pilot.alvo.x", "pilot.alvo.y" }) impl.critic.channels.push_back({ name, "m" });
    impl.critic.channels.push_back({ "pilot.parar", "1" });
    impl.critic.channels.push_back({ "pilot.espera", "s" });
    for (const char* name : { "pilot.viavel.x", "pilot.viavel.y" }) impl.critic.channels.push_back({ name, "m/s" });
    impl.critic.channels.push_back({ "world.tempo", "1" });
    impl.criticSize = impl.critic.size();

    LocomotionBatchSettings3D batchSettings;
    batchSettings.worlds = std::max(1u, settings.environments);
    batchSettings.threads = settings.threads;
    batchSettings.world.runtime = runtimeFor(impl.task);
    impl.batch = std::make_unique<LocomotionBatch3D>(impl.bundle->assets(), batchSettings);
    impl.computeCaps();

    const std::size_t n = impl.batch->size();
    impl.worlds.resize(n);
    impl.worldDrills.assign(n, 0);
    impl.observations.assign(n * impl.observationSize, 0.0f);
    impl.criticObservations.assign(n * impl.criticSize, 0.0f);
    impl.terminalCritic.assign(n * impl.criticSize, 0.0f);
    impl.rewardTerms.assign(n * LocomotionRewardTermCount3D, 0.0f);
    impl.rewards.assign(n, 0.0f);
    impl.terminated.assign(n, 0);
    impl.truncated.assign(n, 0);
    reset(settings.seed);
}

LocomotionTraining3D::~LocomotionTraining3D() = default;

std::size_t LocomotionTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t LocomotionTraining3D::observationSize() const { return m_impl->observationSize; }
std::size_t LocomotionTraining3D::criticObservationSize() const { return m_impl->criticSize; }
std::size_t LocomotionTraining3D::actionSize() const { return m_impl->actionSize; }
std::uint32_t LocomotionTraining3D::workerThreads() const { return m_impl->batch->threads(); }
float LocomotionTraining3D::policyStepSeconds() const { return TickSeconds * TicksPerDecision; }
const LocomotionContract3D& LocomotionTraining3D::contract() const { return *m_impl->contract; }
const RLVectorLayout3D& LocomotionTraining3D::criticLayout() const { return m_impl->critic; }
const LocomotionTask3D& LocomotionTraining3D::task() const { return m_impl->task; }
const AthleteBundle3D& LocomotionTraining3D::athlete() const { return *m_impl->bundle; }
std::span<const std::uint32_t> LocomotionTraining3D::worldDrills() const { return m_impl->worldDrills; }
std::span<const float> LocomotionTraining3D::observations() const { return m_impl->observations; }
std::span<const float> LocomotionTraining3D::criticObservations() const { return m_impl->criticObservations; }
std::span<const float> LocomotionTraining3D::terminalCriticObservations() const { return m_impl->terminalCritic; }
std::span<const float> LocomotionTraining3D::rewardTerms() const { return m_impl->rewardTerms; }
std::span<const float> LocomotionTraining3D::rewards() const { return m_impl->rewards; }
std::span<const std::uint8_t> LocomotionTraining3D::terminated() const { return m_impl->terminated; }
std::span<const std::uint8_t> LocomotionTraining3D::truncated() const { return m_impl->truncated; }
const std::vector<LocomotionEpisode3D>& LocomotionTraining3D::finishedEpisodes() const { return m_impl->finished; }
double LocomotionTraining3D::lastStepMilliseconds() const { return m_impl->lastStepMilliseconds; }

LocomotionTraining3D::Mirror LocomotionTraining3D::mirror() const {
    const Impl& impl = *m_impl;
    const LocomotionMirror3D contract = impl.contract->mirror();
    Mirror out;
    out.observation = contract.observation;
    out.action = contract.action;
    out.critic = contract.observation;
    // (Os canais do critico: o exercicio fica; os vetores trocam o y.)
    for (std::size_t i = impl.observationSize; i < impl.criticSize; ++i) {
        const std::string& name = impl.critic.channels[i].name;
        out.critic.source.push_back(static_cast<std::uint32_t>(i));
        out.critic.scale.push_back(name.size() >= 2 && name.compare(name.size() - 2, 2, ".y") == 0 ? -1.0f : 1.0f);
        out.critic.offset.push_back(0.0f);
    }
    return out;
}

std::vector<float> LocomotionTraining3D::actionCapFractions() const {
    const Impl& impl = *m_impl;
    const LocomotionAuthority3D full = LocomotionAuthority3D::uniform(1.0f, LocomotionCaps3D::full());
    const std::vector<float> scale = impl.contract->physical(std::vector<float>(impl.actionSize, 1.0f), full);
    const std::vector<float> scaleDown = impl.contract->physical(std::vector<float>(impl.actionSize, -1.0f), full);
    std::vector<float> fractions(impl.actionSize, 1.0f);
    for (std::size_t i = 0; i < impl.actionSize; ++i) {
        const float most = std::max(std::abs(scale[i]), std::abs(scaleDown[i]));
        fractions[i] = most > 0.0f ? impl.capMax[i] / most : 1.0f;
    }
    return fractions;
}

void LocomotionTraining3D::updateTask(LocomotionTask3D task) {
    Impl& impl = *m_impl;
    if (task.drills.size() != impl.task.drills.size())
        throw std::runtime_error("treino da locomocao: a tarefa nova tem outros exercicios");
    for (std::size_t d = 0; d < task.drills.size(); ++d)
        if (task.drills[d].id != impl.task.drills[d].id)
            throw std::runtime_error("treino da locomocao: a tarefa nova tem outros exercicios");
    // As ajudas vao para os mundos na criacao (runtimeFor): uma tarefa com
    // outras ajudas pede outro lote, nao uma troca silenciosa.
    if (task.standingAssistPercent != impl.task.standingAssistPercent
        || task.walkingAssistPercent != impl.task.walkingAssistPercent
        || task.feetAssistPercent != impl.task.feetAssistPercent
        || task.getUpAssistPercent != impl.task.getUpAssistPercent)
        throw std::runtime_error("treino da locomocao: a tarefa nova muda as ajudas (abra outro lote)");
    impl.task = std::move(task);
    impl.computeCaps();
}

void LocomotionTraining3D::setAuthority(float footwork, float balance, float pose, float gain) {
    Impl& impl = *m_impl;
    impl.authority.footwork = footwork;
    impl.authority.balance = balance;
    impl.authority.pose = pose;
    impl.authority.gain = gain;
}

void LocomotionTraining3D::setTurnMoveMode(int mode) { m_impl->turnMoveMode = std::clamp(mode, 0, 2); }

bool LocomotionTraining3D::turnMoveActive(std::size_t world) const {
    return world < m_impl->worlds.size() && m_impl->batch->world(world).runtimeSettings().dynamicTurnMove;
}

void LocomotionTraining3D::setDrillWeights(std::span<const float> weights) {
    Impl& impl = *m_impl;
    if (weights.size() != impl.drillCount) throw std::runtime_error("treino da locomocao: um peso por exercicio");
    impl.drillWeights.assign(weights.begin(), weights.end());
}

void LocomotionTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) {
        impl.worlds[i].rng.seed(seed * 0x9E3779B97F4A7C15ull + i * 0xBF58476D1CE4E5B9ull + 1);
        impl.batch->clearFailure(i);
    }
    impl.finished.clear();
    impl.batch->parallel([&](std::size_t i) {
        impl.startEpisode(i);
        std::span<float> obs(impl.observations.data() + i * impl.observationSize, impl.observationSize);
        std::span<float> critic(impl.criticObservations.data() + i * impl.criticSize, impl.criticSize);
        (void)impl.observe(i, obs, critic);
    });
}

void LocomotionTraining3D::step(std::span<const float> actions) {
    Impl& impl = *m_impl;
    if (actions.size() != impl.worlds.size() * impl.actionSize)
        throw std::runtime_error("treino da locomocao: acoes com tamanho errado");
    const auto begin = std::chrono::steady_clock::now();
    impl.finished.clear();
    impl.batch->parallel([&](std::size_t i) {
        impl.stepWorld(i, actions.subspan(i * impl.actionSize, impl.actionSize));
    });
    // Um mundo que lancou excecao: recomeca, e o episodio conta como falha.
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) {
        if (!impl.batch->failed(i)) continue;
        impl.batch->clearFailure(i);
        impl.terminated[i] = 1;
        impl.truncated[i] = 0;
        impl.rewards[i] = 0.0f;
        impl.finishEpisode(i, LocomotionEnd3D::Failure);
        std::copy(impl.criticObservations.begin() + static_cast<std::ptrdiff_t>(i * impl.criticSize),
            impl.criticObservations.begin() + static_cast<std::ptrdiff_t>((i + 1) * impl.criticSize),
            impl.terminalCritic.begin() + static_cast<std::ptrdiff_t>(i * impl.criticSize));
        impl.startEpisode(i);
        std::span<float> obs(impl.observations.data() + i * impl.observationSize, impl.observationSize);
        std::span<float> critic(impl.criticObservations.data() + i * impl.criticSize, impl.criticSize);
        (void)impl.observe(i, obs, critic);
    }
    for (Impl::World& world : impl.worlds)
        if (world.hasEpisode) {
            impl.finished.push_back(world.episode);
            world.hasEpisode = false;
        }
    impl.lastStepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

std::string LocomotionTraining3D::frameLines(std::size_t environment) const {
    const Impl& impl = *m_impl;
    if (environment >= impl.worlds.size()) return {};
    const Impl::World& world = impl.worlds[environment];
    const RagdollState3D& body = impl.batch->world(environment).body();
    const RagdollProfile3D& profile = impl.bundle->profile();
    std::ostringstream out;
    for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        const PhysicsBodyState3D& state = body.links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    for (const RagdollContactPoint3D& contact : body.contacts)
        out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
            << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z << ' '
            << contact.normalImpulseNewtonSeconds << '\n';
    if (world.pilot.hasTarget)
        out << "A " << world.pilot.target.x << ' ' << world.pilot.target.y << ' ' << world.pilot.target.z << ' '
            << impl.task.drills[world.drill].pilot.arriveRadius << '\n';
    const Vec3 wanted = world.command.intent.requestedVelocityWorld;
    const float speed = std::hypot(wanted.x, wanted.y);
    if (speed > 1.0e-4f) out << "W " << wanted.x / speed << ' ' << wanted.y / speed << " 0\n";
    out << "U " << speed << ' ' << std::hypot(world.comVelocity.x, world.comVelocity.y) << '\n';
    out << "V " << world.comVelocity.x << ' ' << world.comVelocity.y << ' ' << world.comVelocity.z << '\n';
    if (impl.head >= 0 && static_cast<std::size_t>(impl.head) < body.links.size()) {
        const Quaternion head = body.links[static_cast<std::size_t>(impl.head)].orientation
            * profile.links[static_cast<std::size_t>(impl.head)].modelOrientation.conjugate();
        const Vec3 gaze = head.rotate({ 1.0f, 0.0f, 0.0f });
        const float planarGaze = std::max(1.0e-6f, std::hypot(gaze.x, gaze.y));
        out << "H " << gaze.x / planarGaze << ' ' << gaze.y / planarGaze << " 0\n";
    }
    out << "L " << world.command.facingYawRadians << '\n';
    return out.str();
}

} // namespace MatterEngine
