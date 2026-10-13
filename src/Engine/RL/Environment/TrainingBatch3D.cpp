#include "Engine/RL/Environment/TrainingBatch3D.hpp"

#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>
#include <thread>

namespace MatterEngine {
namespace {

constexpr float FixedStep = 1.0f / 120.0f;
constexpr float Gravity = 9.81f;
// Distancia entre os mundos de uma cena: longe o bastante para nunca se
// tocarem dentro do raio em que o episodio vale.
constexpr std::size_t KeyBodyCount = 5;
constexpr std::size_t RewardCount = TrainingRewardComponentNames3D.size();
// Os canais do critico alem da observacao: atrito, massa, forca, atraso, o
// empurrao (3) e um contato com o chao por link.
constexpr std::size_t CriticWorldChannels = 7;

std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

float uniform(std::mt19937_64& rng, TrainingRange3D range) {
    if (!(range.maximum > range.minimum)) return range.minimum;
    return std::uniform_real_distribution<float>(range.minimum, range.maximum)(rng);
}

float gaussian(std::mt19937_64& rng, float sigma) {
    if (!(sigma > 0.0f)) return 0.0f;
    return std::normal_distribution<float>(0.0f, sigma)(rng);
}

struct Environment {
    std::uint32_t scene = 0;
    RagdollHandle3D ragdoll;
    Vec3 origin;
    std::mt19937_64 rng;
    float massScale = 1.0f;
    // O episodio: o relogio (que pode comecar sorteado) e as decisoes jogadas.
    std::uint32_t steps = 0;
    std::uint32_t played = 0;
    // A postura pedida neste episodio (o indice em body.postures).
    std::uint32_t posture = 0;
    // O comando de velocidade (x para frente, y para a esquerda, z o giro), o
    // tempo ate o proximo sorteio e o comando fixo de fora (o visualizador).
    Vec3 command;
    float commandTimer = 0.0f;
    bool commandForced = false;
    Vec3 forcedCommand;
    // A fase do relogio de passada (0 a 1; parado fica em 0).
    float gaitPhase = 0.0f;
    // Os relogios de cada pe: ha quanto tempo apoiado e ha quanto no ar.
    std::array<float, 2> footContactTime {};
    std::array<float, 2> footAirTime {};
    // O rumo no comeco do episodio (o desvio para o lado e medido dele).
    float startHeading = 0.0f;
    // O limite deste episodio (no treino, a duracao da tarefa +-20%: sem isso
    // as quedas do comeco agrupam as fases e, sem quedas depois, os fins
    // ficam sincronizados para sempre).
    std::uint32_t limit = 600;
    std::vector<float> previousAction;
    std::vector<float> action;
    // Com professor: a soma, nos passos de fisica da decisao, do alvo que daria
    // o torque dele (em unidades da acao).
    std::vector<float> teacherSum;
    std::vector<RagdollDriveTarget3D> targets;
    std::vector<RagdollDriveTarget3D> previousTargets;
    std::array<Vec3, 2> footStart {};
    std::array<Quaternion, 2> footStartOrientation {};
    float friction = 1.0f;
    float strength = 1.0f;
    bool delayed = false;
    // A queda por contato vista em qualquer subpasso (o estado publicado so
    // tem os contatos do ultimo).
    int firstContactLink = -1;
    // Empurrao (H2): tempo ate o proximo, quanto falta do atual, a forca.
    float pushTimer = 0.0f;
    float pushRemaining = 0.0f;
    Vec3 pushForce;
    std::uint32_t pushLink = 0;
    // Acumuladores do resumo.
    double episodeReturn = 0.0;
    double comSum[2] {};
    double comSquares[2] {};
    float footDriftMax = 0.0f;
    float footTurnMax = 0.0f;
    double actionRateSum = 0.0;
    double effortSum = 0.0;
    double jointSpeedSum = 0.0;
    double uprightSum = 0.0;
    double trunkTurnSquares = 0.0;
    double poseErrorSquares = 0.0;
    double velocityErrorSquares = 0.0;
    double yawErrorSquares = 0.0;
    double trackingSum = 0.0;
    std::uint32_t movingSteps = 0;
    // A imitacao: onde a referencia esta e o erro das juntas somado.
    ReferenceCursor3D cursor;
    double imitationErrorSquares = 0.0;
    // O erro do rumo pedido (com o comando de olhar), somado por decisao.
    double facingErrorSquares = 0.0;
    std::uint32_t facingSteps = 0;
    // O alvo: onde ele esta (no chao), de onde o trecho ate ele saiu e quando,
    // a direcao do trecho (rad), quantos nasceram e quantos ele alcancou.
    Vec3 target;
    Vec3 targetFrom;
    float targetSince = 0.0f;
    float targetDirection = 0.0f;
    std::uint32_t targetCount = 0;
    std::uint32_t targetsReached = 0;
    float firstTargetSeconds = -1.0f;
    double targetSpeedSum = 0.0;
    // O mais perto que ele chegou do alvo de agora (o "passou do alvo").
    float targetClosest = 1e9f;
    // O planejador (biblioteca): o indice do mundo (as listas da avaliacao), ha
    // quanto tempo no clipe de agora, o giro pedido (rad/s) e para onde olhar (o
    // rumo no mundo; a camera do jogo), com o quanto ele gira agora.
    std::uint32_t index = 0;
    float clipSince = 0.0f;
    float turnRate = 0.0f;
    float facingCommand = 0.0f;
    float facingRate = 0.0f;
    float facingTimer = 0.0f;
    // O comando vale neste episodio (target.facingFraction) e o rumo do que
    // fica virado para ele (a pelve ou o peito), medido na ultima decisao.
    bool facingActive = false;
    float facingHeading = 0.0f;
    // Os proximos alvos ja no mapa (target.lookahead) com o rumo do trecho que
    // chega em cada um, e o percurso fixo da avaliacao (o rumo do comeco).
    std::array<Vec3, 3> upcoming {};
    std::array<float, 3> upcomingDirection {};
    float courseHeading = 0.0f;
    // Os ultimos alvos alcancados (o mais novo primeiro), para o visualizador.
    std::array<Vec3, 3> reachedTargets {};
    std::uint32_t reachedCount = 0;
};

struct Scene {
    std::unique_ptr<PhysicsScene3D> physics;
    std::vector<std::uint32_t> environments;
    std::vector<TrainingEpisode3D> finished;
};

// O que a observacao e a recompensa leem de um corpo, uma vez por passo.
struct BodyReading {
    Quaternion pelvisFrame;
    Quaternion headingInverse;
    Vec3 com;
    Vec3 comVelocity;
    std::array<Vec3, 2> footCenter {};
    std::array<bool, 2> footContact {};
    std::array<float, 2> footForce {};
    std::uint32_t groundMask = 0;
    bool valid = true;
};

} // namespace

const char* trainingEndReasonName3D(TrainingEndReason3D reason) {
    switch (reason) {
        case TrainingEndReason3D::None: return "nenhum";
        case TrainingEndReason3D::FallContact: return "queda_contato";
        case TrainingEndReason3D::FallHeight: return "queda_altura";
        case TrainingEndReason3D::Numeric: return "numerico";
        case TrainingEndReason3D::TimeLimit: return "tempo";
        case TrainingEndReason3D::Boundary: return "limite";
        case TrainingEndReason3D::TargetMissed: return "alvo_perdido";
    }
    return "?";
}

struct TrainingBatch3D::Impl {
    TrainingBatchSettings3D settings;
    RLBody3D body;
    TrainingTask3D task;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    std::vector<Scene> scenes;
    std::vector<Environment> environments;
    RLVectorLayout3D observationLayout;
    RLVectorLayout3D criticLayout;
    RLVectorLayout3D actionLayout;
    std::vector<float> observations;
    std::vector<float> critic;
    std::vector<float> rewards;
    std::vector<float> components;
    std::vector<float> terminalCritic;
    std::vector<std::uint8_t> terminated;
    std::vector<std::uint8_t> truncated;
    std::vector<TrainingEpisode3D> finished;
    std::vector<std::uint32_t> pushLinks;
    // As posturas da tarefa (indices em body.postures) com a fatia acumulada
    // de cada uma, e a forcada (-1 = a mistura).
    std::vector<std::uint32_t> taskPostures;
    std::vector<float> postureCumulative;
    // As posturas so de parado (indices em body.postures).
    std::vector<std::uint8_t> standingOnly;
    int forcedPosture = -1;
    // Os eixos do tronco e dos bracos (andando, a postura pedida vale so neles).
    std::vector<std::uint8_t> upperDof;
    // A referencia de passada: o eixo de cada junta pedida, a perna e a escala.
    struct ReferenceDof {
        std::size_t dof = 0;
        std::uint32_t leg = 0;
        float scale = 0.0f;
    };
    std::vector<ReferenceDof> referenceDofs;
    // A acao do professor no ultimo step (mundos x eixos).
    std::vector<float> teacherLabels;
    std::array<std::uint32_t, KeyBodyCount> keyBodies {};
    // A referencia da imitacao (vazia sem a secao imitation).
    ReferenceMotion3D reference;
    std::uint32_t workers = 1;
    std::uint32_t maximumSteps = 600;
    float lastStepMilliseconds = 0.0f;
    std::uint64_t physicsSteps = 0;

    [[nodiscard]] std::size_t actionSize() const { return body.dofs.size(); }
    [[nodiscard]] std::size_t observationSize() const { return observationLayout.size(); }
    [[nodiscard]] std::size_t criticSize() const { return criticLayout.size(); }
    [[nodiscard]] bool training() const { return !settings.evaluation; }
    [[nodiscard]] bool teaching() const { return settings.teacher != nullptr; }
    [[nodiscard]] bool imitating() const { return task.imitation.enabled && !reference.segments.empty(); }
    [[nodiscard]] bool targeting() const { return task.target.enabled; }
    // O mundo visto pelo professor.
    [[nodiscard]] TrainingTeacherWorld3D teacherWorld(std::uint32_t index) {
        const Environment& env = environments[index];
        return { index, scenes[env.scene].physics.get(), env.ragdoll, env.origin, env.startHeading };
    }
    // A acao do professor (o alvo que daria o torque dele, em unidades da
    // acao) a partir do estado depois de um passo de fisica, somada em sum.
    void accumulateTeacher(const RagdollState3D& state, std::vector<float>& sum) const;

    void buildLayouts();
    void buildReference();
    // O primeiro alvo do episodio (na direcao em que a referencia anda) e o
    // seguinte, distance adiante do alcancado.
    void placeFirstTarget(Environment& env, const RagdollState3D& state);
    void placeNextTarget(Environment& env);
    // Biblioteca: o alvo a partir de um ponto e de um rumo (o k-esimo do
    // episodio, para as listas da avaliacao), o planejador (o clipe que anda mais
    // perto da direcao do alvo; force troca sem histerese) e o giro pedido.
    void placeLibraryTarget(Environment& env, Vec3 from, float facing, std::uint32_t k);
    void drawLibraryTarget(Environment& env, Vec3 from, float facing, std::uint32_t k, Vec3& point, float& direction);
    void plan(Environment& env, Vec3 position, float facing, bool force);
    [[nodiscard]] float wantedTurn(const Environment& env, Vec3 position, float facing) const;
    [[nodiscard]] Vec3 aimPoint(const Environment& env) const;
    // O rumo no chao do link que fica virado para o comando de olhar (rad).
    [[nodiscard]] float facingLinkHeading(const RagdollState3D& state) const;
    [[nodiscard]] std::vector<ReferenceDof> resolveReference(const TrainingTask3D::Gait& gait) const;
    void resolvePostures();
    [[nodiscard]] std::uint32_t choosePosture(Environment& env) const;
    void updateCommand(Environment& env, bool start);
    void createWorld();
    void configureEpisodeLength();
    void resetEnvironment(std::uint32_t index);
    void writeTargets(Environment& env, std::span<const float> action);
    [[nodiscard]] BodyReading read(const RagdollState3D& state) const;
    void writeObservation(const Environment& env, const RagdollState3D& state, const BodyReading& reading,
        float* actor, float* criticOut, bool noise);
    float reward(Environment& env, const RagdollState3D& state, const BodyReading& reading, bool fell,
        float* componentsOut);
    void finishEpisode(Scene& scene, std::uint32_t index, TrainingEndReason3D reason, const BodyReading& reading);
    void stepScene(Scene& scene, std::span<const float> actions);
};

void TrainingBatch3D::Impl::buildLayouts() {
    const auto vec3 = [](RLVectorLayout3D& layout, const std::string& name, const char* unit) {
        for (const char* axis : { ".x", ".y", ".z" }) layout.channels.push_back({ name + axis, unit });
    };
    RLVectorLayout3D& o = observationLayout;
    o.id = "matter.rl.observation";
    o.version = 1;
    o.channels.push_back({ "pelvis.height_error", "m/0.3" });
    vec3(o, "pelvis.gravity_local", "unit");
    vec3(o, "pelvis.linear_velocity_heading", "m/s/2");
    vec3(o, "pelvis.angular_velocity_local", "rad/s/4");
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".position_from_base", "rad" });
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".velocity", "rad/s/10" });
    for (const std::uint32_t link : keyBodies) vec3(o, body.profile.links[link].id + ".position_heading", "m");
    vec3(o, "LeftFoot.gravity_local", "unit");
    vec3(o, "RightFoot.gravity_local", "unit");
    vec3(o, "com.position_heading", "m");
    vec3(o, "com.velocity_heading", "m/s/2");
    o.channels.push_back({ "com.support_offset.x", "m/0.2" });
    o.channels.push_back({ "com.support_offset.y", "m/0.2" });
    o.channels.push_back({ "LeftFoot.contact", "0/1" });
    o.channels.push_back({ "RightFoot.contact", "0/1" });
    o.channels.push_back({ "LeftFoot.normal_force", "peso" });
    o.channels.push_back({ "RightFoot.normal_force", "peso" });
    for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ dof.name + ".previous_action", "acao" });
    // O comando: a postura pedida, um canal por postura alem da base (todos
    // zero = a base). No fim: uma politica sem ele le o comeco igual.
    for (std::size_t p = 1; p < body.postures.size(); ++p)
        o.channels.push_back({ "command.posture." + body.postures[p].id, "0/1" });
    // O comando de velocidade (zero parado): para frente e para a esquerda no
    // referencial do rumo, e o giro.
    o.channels.push_back({ "command.velocity.forward", "m/s/3" });
    o.channels.push_back({ "command.velocity.lateral", "m/s/3" });
    o.channels.push_back({ "command.velocity.yaw", "rad/s/2" });
    // O relogio de passada (zero parado): o seno e o cosseno da fase.
    o.channels.push_back({ "command.gait.sin", "1" });
    o.channels.push_back({ "command.gait.cos", "1" });
    // A imitacao (so com a secao): o trecho e a fase da referencia e as juntas
    // dela no fim da decisao (o alvo de pose do passo).
    if (imitating()) {
        o.channels.push_back({ "reference.start", "0/1" });
        o.channels.push_back({ "reference.progress", "0..1" });
        o.channels.push_back({ "reference.loop.sin", "1" });
        o.channels.push_back({ "reference.loop.cos", "1" });
        for (const RLBodyDof3D& dof : body.dofs) o.channels.push_back({ "reference." + dof.name, "rad" });
    }
    // O alvo (so com a secao): a direcao no referencial do rumo e a distancia.
    if (targeting()) {
        o.channels.push_back({ "target.direction.x", "unit" });
        o.channels.push_back({ "target.direction.y", "unit" });
        o.channels.push_back({ "target.distance", "m/10" });
        // Os proximos (os varios pontos), do mesmo jeito, em ordem.
        for (std::uint32_t i = 0; i < task.target.lookahead; ++i) {
            const std::string name = "target.next" + std::to_string(i + 1);
            o.channels.push_back({ name + ".direction.x", "unit" });
            o.channels.push_back({ name + ".direction.y", "unit" });
            o.channels.push_back({ name + ".distance", "m/10" });
        }
    }
    // A biblioteca: o clipe de agora (um canal por clipe) e o giro pedido.
    if (imitating() && reference.library) {
        for (const ReferenceSegment3D& segment : reference.segments)
            o.channels.push_back({ "reference.clip." + segment.id, "0/1" });
        o.channels.push_back({ "target.turn_rate", "rad/s/2" });
        if (task.target.facing) {
            o.channels.push_back({ "command.facing.sin", "1" });
            o.channels.push_back({ "command.facing.cos", "1" });
            o.channels.push_back({ "command.facing.active", "0/1" });
        }
    }

    RLVectorLayout3D& c = criticLayout;
    c = o;
    c.id = "matter.rl.critic";
    c.channels.push_back({ "world.friction_scale", "x" });
    c.channels.push_back({ "body.mass_scale", "x" });
    c.channels.push_back({ "body.strength_scale", "x" });
    c.channels.push_back({ "body.action_delayed", "0/1" });
    vec3(c, "push.force_heading", "N/200");
    for (const RagdollLinkDefinition3D& link : body.profile.links)
        c.channels.push_back({ link.id + ".ground_contact", "0/1" });

    RLVectorLayout3D& a = actionLayout;
    a.id = "matter.rl.action";
    a.version = 1;
    for (const RLBodyDof3D& dof : body.dofs) a.channels.push_back({ dof.name + ".target", "acao" });
}

void TrainingBatch3D::Impl::buildReference() {
    reference = {};
    if (!task.imitation.enabled) return;
    const bool library = task.imitation.library;
    std::vector<ReferenceClip3D> clips;
    for (std::size_t i = 0; i < task.imitation.clips.size(); ++i)
        clips.push_back({ task.imitation.clips[i].id, task.imitation.clips[i].path,
            library || i + 1 == task.imitation.clips.size() });
    reference = buildReferenceMotion3D(body, clips, FixedStep, library);
}

// Um alvo em volta de from, a partir do rumo facing (o k-esimo do episodio): no
// treino sorteado, na avaliacao pelas listas ou pelo percurso do mundo.
void TrainingBatch3D::Impl::drawLibraryTarget(Environment& env, Vec3 from, float facing, std::uint32_t k, Vec3& point,
    float& direction) {
    const TrainingTask3D::Target& t = task.target;
    constexpr float Radians = std::numbers::pi_v<float> / 180.0f;
    if (!training() && !t.evaluationCourses.empty()) {
        const auto& points = t.evaluationCourses[(t.evaluationPerWorld ? env.index : 0u) % t.evaluationCourses.size()].points;
        const std::size_t n = points.size();
        const auto at = [&](std::size_t i) {
            const float laps = static_cast<float>(i / n);
            const float x = points[i % n][0] + laps * points[n - 1][0], y = points[i % n][1] + laps * points[n - 1][1];
            const float c = std::cos(env.courseHeading), s = std::sin(env.courseHeading);
            return env.origin + Vec3 { c * x - s * y, s * x + c * y, 0.0f };
        };
        point = at(k);
        const Vec3 previous = k == 0 ? Vec3 { env.origin.x, env.origin.y, 0.0f } : at(k - 1);
        direction = std::atan2(point.y - previous.y, point.x - previous.x);
        point.z = 0.0f;
        return;
    }
    float angle = 0.0f, distance = t.distance;
    if (training()) {
        const float spread = (k == 0 ? t.firstAngleDegrees : t.nextAngleDegrees) * Radians;
        angle = std::uniform_real_distribution<float>(-spread, spread)(env.rng);
        if (t.distanceRange.maximum > 0.0f) distance = uniform(env.rng, t.distanceRange);
    } else {
        const std::size_t slot = (t.evaluationPerWorld ? env.index : 0u) + k;
        angle = t.evaluationAngles[slot % t.evaluationAngles.size()] * Radians;
        if (!t.evaluationDistances.empty()) distance = t.evaluationDistances[slot % t.evaluationDistances.size()];
    }
    direction = facing + angle;
    point = Vec3 { from.x, from.y, 0.0f } + Vec3 { std::cos(direction), std::sin(direction), 0.0f } * distance;
}

// O k-esimo alvo do episodio. Sem lookahead ele nasce em volta de from (onde o
// corpo chegou) pelo rumo do corpo; com lookahead, o de agora e o primeiro da
// fila e um novo entra no fim, saindo do ultimo pelo rumo do trecho dele.
void TrainingBatch3D::Impl::placeLibraryTarget(Environment& env, Vec3 from, float facing, std::uint32_t k) {
    const std::uint32_t ahead = task.target.lookahead;
    if (k == 0) env.courseHeading = facing;
    if (ahead == 0 || k == 0) {
        env.targetFrom = { from.x, from.y, 0.0f };
        drawLibraryTarget(env, from, facing, k, env.target, env.targetDirection);
        Vec3 last = env.target;
        float lastDirection = env.targetDirection;
        for (std::uint32_t i = 0; i < ahead; ++i) {
            drawLibraryTarget(env, last, lastDirection, k + 1 + i, env.upcoming[i], env.upcomingDirection[i]);
            last = env.upcoming[i];
            lastDirection = env.upcomingDirection[i];
        }
    } else {
        env.targetFrom = env.target;
        env.target = env.upcoming[0];
        env.targetDirection = env.upcomingDirection[0];
        for (std::uint32_t i = 0; i + 1 < ahead; ++i) {
            env.upcoming[i] = env.upcoming[i + 1];
            env.upcomingDirection[i] = env.upcomingDirection[i + 1];
        }
        const Vec3 last = ahead > 1 ? env.upcoming[ahead - 2] : env.target;
        const float lastDirection = ahead > 1 ? env.upcomingDirection[ahead - 2] : env.targetDirection;
        drawLibraryTarget(env, last, lastDirection, k + ahead, env.upcoming[ahead - 1], env.upcomingDirection[ahead - 1]);
    }
    env.targetClosest = 1e9f;
    env.targetCount = k + 1;
}

float TrainingBatch3D::Impl::facingLinkHeading(const RagdollState3D& state) const {
    const std::uint32_t link = task.target.facingLink == "chest" ? body.upperChest : 0u;
    const Quaternion frame =
        (state.links[link].orientation * body.profile.links[link].modelOrientation.conjugate()).normalized();
    const Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

// A mira do planejador e do giro: o centro do alvo, ou, com o tracado de
// corrida, o ponto do disco do lado do proximo alvo.
Vec3 TrainingBatch3D::Impl::aimPoint(const Environment& env) const {
    const TrainingTask3D::Target& t = task.target;
    if (!(t.racingLine > 0.0f) || t.lookahead == 0) return env.target;
    const Vec3 toward { env.upcoming[0].x - env.target.x, env.upcoming[0].y - env.target.y, 0.0f };
    const float length = std::hypot(toward.x, toward.y);
    if (length < 1.0e-4f) return env.target;
    return env.target + toward * (t.racingLine * t.radius / length);
}

float TrainingBatch3D::Impl::wantedTurn(const Environment& env, Vec3 position, float facing) const {
    const TrainingTask3D::Target& t = task.target;
    // Olhando para o comando (a camera): o giro so corrige o rumo do corpo.
    if (env.facingActive) {
        float error = env.facingCommand - env.facingHeading;
        error = std::atan2(std::sin(error), std::cos(error));
        return std::clamp(t.turnGain * error, -t.turnMaximum, t.turnMaximum);
    }
    const Vec3 aim = aimPoint(env);
    const float dx = aim.x - position.x, dy = aim.y - position.y;
    if (dx * dx + dy * dy < 1.0e-6f) return 0.0f;
    const ReferenceSegment3D& segment =
        reference.segments[std::min<std::size_t>(env.cursor.segment, reference.segments.size() - 1)];
    float error = std::atan2(dy, dx) - facing - segment.travelRelative;
    error = std::atan2(std::sin(error), std::cos(error));
    return std::clamp(t.turnGain * error, -t.turnMaximum, t.turnMaximum);
}

void TrainingBatch3D::Impl::plan(Environment& env, Vec3 position, float facing, bool force) {
    const TrainingTask3D::Imitation::Planner& planner = task.imitation.planner;
    const float base = env.facingActive ? env.facingCommand : facing;
    const Vec3 aim = aimPoint(env);
    const float dx = aim.x - position.x, dy = aim.y - position.y;
    float angle = std::atan2(dy, dx) - base;
    angle = std::atan2(std::sin(angle), std::cos(angle));
    // O clipe que anda mais perto da direcao da mira (a partir de para onde ele
    // olha); o de agora leva a histerese.
    const float hysteresis = force ? 0.0f : planner.hysteresisDegrees * std::numbers::pi_v<float> / 180.0f;
    std::uint32_t best = env.cursor.segment;
    float bestCost = std::numeric_limits<float>::max();
    for (std::uint32_t c = 0; c < reference.segments.size(); ++c) {
        float error = angle - reference.segments[c].travelRelative;
        error = std::abs(std::atan2(std::sin(error), std::cos(error)));
        if (!force && c == env.cursor.segment) error -= hysteresis;
        if (error < bestCost) {
            bestCost = error;
            best = c;
        }
    }
    if (!force && best != env.cursor.segment && env.clipSince < planner.minimumSeconds) best = env.cursor.segment;
    if (force) {
        env.cursor = { best, 0 };
        env.clipSince = 0.0f;
    } else if (best != env.cursor.segment) {
        env.cursor = switchReference3D(reference, env.cursor, best);
        env.clipSince = 0.0f;
    }
    env.turnRate = wantedTurn(env, position, facing);
}

void TrainingBatch3D::Impl::placeFirstTarget(Environment& env, const RagdollState3D& state) {
    const TrainingTask3D::Target& t = task.target;
    const PhysicsBodyState3D& pelvis = state.links.front();
    const Quaternion frame = (pelvis.orientation * body.profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
    // A direcao em que a referencia anda a partir deste quadro (em relacao ao
    // rumo da pelve); sem imitacao, a frente da pelve.
    float direction = std::atan2(forward.y, forward.x);
    if (imitating()) {
        const ReferenceSegment3D& segment = reference.segments[std::min<std::size_t>(env.cursor.segment,
            reference.segments.size() - 1)];
        direction += segment.travelHeading - referenceFrame3D(reference, env.cursor).heading;
    }
    constexpr float Radians = std::numbers::pi_v<float> / 180.0f;
    const float spread = t.firstAngleDegrees * Radians;
    direction += training() ? std::uniform_real_distribution<float>(-spread, spread)(env.rng)
                            : t.evaluationAngles.front() * Radians;
    env.targetFrom = { pelvis.position.x, pelvis.position.y, 0.0f };
    env.targetDirection = direction;
    env.target = env.targetFrom + Vec3 { std::cos(direction), std::sin(direction), 0.0f } * t.distance;
    env.targetClosest = 1e9f;
    env.targetSince = 0.0f;
    env.targetCount = 1;
    env.targetsReached = 0;
    env.reachedCount = 0;
    env.firstTargetSeconds = -1.0f;
    env.targetSpeedSum = 0.0;
}

void TrainingBatch3D::Impl::placeNextTarget(Environment& env) {
    const TrainingTask3D::Target& t = task.target;
    constexpr float Radians = std::numbers::pi_v<float> / 180.0f;
    const float spread = t.nextAngleDegrees * Radians;
    const std::size_t index = std::min<std::size_t>(env.targetCount, t.evaluationAngles.size() - 1);
    env.targetDirection += training() ? std::uniform_real_distribution<float>(-spread, spread)(env.rng)
                                      : t.evaluationAngles[index] * Radians;
    env.targetFrom = env.target;
    env.target = env.targetFrom + Vec3 { std::cos(env.targetDirection), std::sin(env.targetDirection), 0.0f } * t.distance;
    env.targetClosest = 1e9f;
    ++env.targetCount;
}

void TrainingBatch3D::Impl::configureEpisodeLength() {
    const float seconds = settings.evaluation ? settings.evaluationEpisodeSeconds : task.episodeSeconds;
    const float stepSeconds = static_cast<float>(task.actionRepeat) * FixedStep;
    maximumSteps = std::max(1u, static_cast<std::uint32_t>(std::lround(seconds / stepSeconds)));
}

void TrainingBatch3D::Impl::createWorld() {
    const std::uint32_t count = settings.environmentCount;
    const std::uint32_t perScene = std::max(1u, settings.environmentsPerScene);
    const std::uint32_t sceneCount = (count + perScene - 1) / perScene;
    scenes.resize(sceneCount);
    environments.resize(count);

    std::uint64_t seedState = settings.seed ^ 0xA5A5A5A55A5A5A5Aull;
    for (std::uint32_t s = 0; s < sceneCount; ++s) {
        Scene& scene = scenes[s];
        const std::uint32_t first = s * perScene;
        const std::uint32_t last = std::min(count, first + perScene);
        PhysicsSceneSettings3D sceneSettings;
        sceneSettings.gravity = { 0.0f, 0.0f, -Gravity };
        sceneSettings.workerThreadCount = 1;
        sceneSettings.runJobsInline = true;
        sceneSettings.maximumBodies = 64u + 32u * perScene;
        sceneSettings.maximumBodyPairs = 2048u * perScene;
        sceneSettings.maximumContactConstraints = 1024u * perScene;
        sceneSettings.scratchBufferSizeBytes = (2u + perScene) * 1024u * 1024u;
        sceneSettings.windShelterDistanceMeters = 0.0f;
        scene.physics = engine->createScene(sceneSettings, materials);

        // O chao: uma caixa estatica grande sob os mundos da cena (topo em z = 0),
        // com metade da distancia entre eles em volta de cada um.
        const float spacing = task.worldSpacingMeters;
        const float halfLength = 0.5f * spacing * static_cast<float>(last - first) + std::max(20.0f, 0.5f * spacing);
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { halfLength, std::max(20.0f, 0.5f * spacing), 0.5f };
        shape.materialId = task.groundMaterial;
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.5f };
        ground.materialId = task.groundMaterial;
        (void)scene.physics->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));

        for (std::uint32_t index = first; index < last; ++index) {
            Environment& env = environments[index];
            env.scene = s;
            env.index = index;
            env.origin = { (static_cast<float>(index - first) - 0.5f * static_cast<float>(last - first - 1)) * spacing,
                0.0f, 0.0f };
            std::uint64_t envSeed = seedState + index * 0x9E3779B97F4A7C15ull;
            env.rng.seed(splitmix64(envSeed));
            // A massa varia por mundo (fica a vida toda dele): o corpo nao e
            // recriado entre episodios.
            env.massScale = training() && !teaching() ? uniform(env.rng, task.randomization.mass) : 1.0f;
            RagdollProfile3D profile = body.profile;
            profile.totalMassKg *= env.massScale;
            RagdollSpawnDefinition3D spawn;
            spawn.entityId = index + 1;
            spawn.pelvisPosition = env.origin + Vec3 { 0.0f, 0.0f, body.standingPelvisHeight + 0.05f };
            spawn.active = true;
            spawn.rigidityPercent = 0.0f;
            spawn.solverPositionIterations = body.solverPositionIterations;
            spawn.solverVelocityIterations = body.solverVelocityIterations;
            if (settings.solverPositionIterations >= 0)
                spawn.solverPositionIterations = static_cast<std::uint32_t>(settings.solverPositionIterations);
            if (settings.solverVelocityIterations >= 0)
                spawn.solverVelocityIterations = static_cast<std::uint32_t>(settings.solverVelocityIterations);
            env.ragdoll = scene.physics->createRagdoll(profile, spawn);
            env.previousAction.assign(actionSize(), 0.0f);
            env.action.assign(actionSize(), 0.0f);
            env.targets.resize(actionSize());
            env.previousTargets.resize(actionSize());
            env.teacherSum.assign(teaching() ? actionSize() : 0, 0.0f);
            scene.environments.push_back(index);
            if (teaching()) settings.teacher->attach(teacherWorld(index));
        }
        scene.finished.reserve(perScene);
    }
}

void TrainingBatch3D::Impl::writeTargets(Environment& env, std::span<const float> action) {
    // A acao como correcao da referencia (residualActions): a partir da pose e
    // da velocidade dela no fim desta decisao.
    const ReferenceFrame3D* reference = imitating() && task.imitation.residualActions
        ? &referenceFrame3D(this->reference, advanceReference3D(this->reference, env.cursor, task.actionRepeat))
        : nullptr;
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        RagdollDriveTarget3D& target = env.targets[d];
        target.linkIndex = dof.linkIndex;
        target.axis = dof.axis;
        const float a = action.empty() ? 0.0f : std::clamp(action[d], -RLActionLimit, RLActionLimit);
        target.positionRadians = reference
            ? std::clamp(reference->dofRadians[d] + dof.actionScale * a, dof.minimumRadians, dof.maximumRadians)
            : dof.target(a);
        target.velocityRadiansPerSecond = reference ? reference->dofVelocities[d] : 0.0f;
        target.feedforwardTorqueNewtonMeters = 0.0f;
        target.stiffnessScale = dof.stiffnessScale;
        target.dampingScale = dof.dampingScale;
        target.maximumTorqueScale = dof.maximumTorqueScale * env.strength;
        target.gravityCompensationScale = 0.0f;
    }
}

std::vector<TrainingBatch3D::Impl::ReferenceDof> TrainingBatch3D::Impl::resolveReference(
    const TrainingTask3D::Gait& gait) const {
    std::vector<ReferenceDof> resolved;
    for (const TrainingTask3D::ReferenceJoint& joint : gait.referenceJoints) {
        std::size_t found = body.dofs.size();
        for (std::size_t d = 0; d < body.dofs.size(); ++d)
            if (body.dofs[d].name == joint.dof) found = d;
        if (found == body.dofs.size())
            throw std::runtime_error("lote de RL: a referencia de passada pede " + joint.dof + ", que o corpo nao tem");
        resolved.push_back({ found, joint.leg, joint.scale });
    }
    return resolved;
}

void TrainingBatch3D::Impl::resolvePostures() {
    taskPostures.clear();
    postureCumulative.clear();
    standingOnly.assign(body.postures.size(), 0);
    float total = 0.0f;
    for (const TrainingPostureWeight3D& entry : task.postures) {
        const int index = findRLPosture3D(body, entry.id);
        if (index < 0) throw std::runtime_error("lote de RL: o corpo nao tem a postura " + entry.id);
        taskPostures.push_back(static_cast<std::uint32_t>(index));
        if (entry.standingOnly) standingOnly[static_cast<std::size_t>(index)] = 1;
        total += std::max(0.0f, entry.weight);
        postureCumulative.push_back(total);
    }
}

std::uint32_t TrainingBatch3D::Impl::choosePosture(Environment& env) const {
    if (forcedPosture >= 0) return static_cast<std::uint32_t>(forcedPosture);
    if (!training() || taskPostures.size() == 1) return taskPostures.front();
    const float pick = std::uniform_real_distribution<float>(0.0f, postureCumulative.back())(env.rng);
    for (std::size_t i = 0; i < taskPostures.size(); ++i)
        if (pick < postureCumulative[i]) return taskPostures[i];
    return taskPostures.back();
}

// O comando de velocidade: o roteiro da bateria, o fixo de fora ou, no treino,
// sorteado a cada intervalo (parado numa fatia; cada direcao ate o limite de
// agora do curriculo). Comandos pequenos (menos de 0,3 m/s, ambiguos com
// ficar parado) viram parado.
void TrainingBatch3D::Impl::updateCommand(Environment& env, bool start) {
    const float stepSeconds = static_cast<float>(task.actionRepeat) * FixedStep;
    // Com professor, o comando e o que ele tenta (a cada passo, em stepScene).
    if (teaching()) {
        if (start) env.command = {};
        return;
    }
    if (env.commandForced) {
        env.command = env.forcedCommand;
        return;
    }
    if (settings.evaluation) {
        env.command = {};
        const float t = static_cast<float>(env.played) * stepSeconds;
        for (const TrainingCommandStep3D& step : settings.commandSchedule)
            if (t >= step.seconds) env.command = { step.forward, step.lateral, step.yawRate };
        return;
    }
    if (!start && (env.commandTimer -= stepSeconds) > 0.0f) return;
    const TrainingTask3D::Commands& c = task.commands;
    env.commandTimer = uniform(env.rng, c.resampleSeconds);
    const auto limit = [](float maximum, float current) { return current < 0.0f ? maximum : std::min(maximum, current); };
    const float forward = limit(c.forward, c.limits.forward), backward = limit(c.backward, c.limits.backward);
    const float lateral = limit(c.lateral, c.limits.lateral), yaw = limit(c.yawRate, c.limits.yawRate);
    env.command = {};
    if (forward + backward + lateral + yaw <= 0.0f || standingOnly[env.posture]
        || std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng) < c.standingFraction)
        return;
    // Um pedido pequeno demais para andar (abaixo de 0,3 m/s e de 0,2 rad/s)
    // e sorteado de novo: parado so vem da fatia standingFraction.
    for (int attempt = 0; attempt < 8; ++attempt) {
        const Vec3 command { std::uniform_real_distribution<float>(-backward, forward)(env.rng),
            lateral > 0.0f ? std::uniform_real_distribution<float>(-lateral, lateral)(env.rng) : 0.0f,
            yaw > 0.0f ? std::uniform_real_distribution<float>(-yaw, yaw)(env.rng) : 0.0f };
        if (std::hypot(command.x, command.y) < 0.3f && std::abs(command.z) < 0.2f) continue;
        env.command = command;
        return;
    }
}

void TrainingBatch3D::Impl::resetEnvironment(std::uint32_t index) {
    Environment& env = environments[index];
    Scene& scene = scenes[env.scene];
    const TrainingTask3D::InitialState& start = task.initialState;
    const float noise = 1.0f;

    // A postura do episodio com variacao pequena, girada no rumo sorteado e
    // apoiada no chao (o ponto mais baixo dos pes 2 mm acima dele).
    env.posture = choosePosture(env);
    const RLPosture3D& posture = body.postures[env.posture];
    // O comando do comeco (o roteiro da bateria conta do zero).
    env.steps = 0;
    env.played = 0;
    updateCommand(env, true);
    std::vector<PhysicsBodyState3D> links;
    if (imitating()) {
        // Pela referencia (o RSI do DeepMimic): do primeiro quadro (o parado da
        // partida) ou de uma fase sorteada do ciclo, com a pose e as
        // velocidades do clipe mais o ruido do comeco.
        const std::uint32_t last = static_cast<std::uint32_t>(reference.segments.size() - 1);
        const float startYaw = start.randomYaw
            ? std::uniform_real_distribution<float>(-std::numbers::pi_v<float>, std::numbers::pi_v<float>)(env.rng)
            : 0.0f;
        if (reference.library) {
            // Biblioteca: o alvo primeiro (em volta, pelo rumo sorteado), o
            // clipe que o planejador escolhe para ele e uma fase dele.
            env.targetsReached = 0;
            env.reachedCount = 0;
            env.firstTargetSeconds = -1.0f;
            env.targetSpeedSum = 0.0;
            env.targetSince = 0.0f;
            // Para onde olhar: o rumo do corpo no comeco (a camera atras dele),
            // valendo numa fracao dos episodios de treino (na avaliacao, o que a
            // bateria pedir).
            env.facingCommand = startYaw;
            env.facingHeading = startYaw;
            env.facingRate = 0.0f;
            env.facingTimer = 0.0f;
            env.facingActive = task.target.facing
                && (training() ? std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng) < task.target.facingFraction
                               : settings.facing != 0);
            placeLibraryTarget(env, env.origin, startYaw, 0);
            plan(env, env.origin, startYaw, true);
            const std::uint32_t frames = static_cast<std::uint32_t>(reference.segments[env.cursor.segment].frames.size());
            env.cursor.frame = training() ? std::uniform_int_distribution<std::uint32_t>(0, frames - 1)(env.rng) : 0u;
            env.turnRate = wantedTurn(env, env.origin, startYaw);
        } else {
            const bool fromStart = !training()
                || std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng) < task.imitation.standingStart;
            env.cursor = { 0, 0 };
            if (!fromStart) {
                const std::uint32_t frames = static_cast<std::uint32_t>(reference.segments[last].frames.size());
                env.cursor = { last, std::uniform_int_distribution<std::uint32_t>(0, frames - 1)(env.rng) };
            }
        }
        ReferenceFrame3D frame = referenceFrame3D(reference, env.cursor);
        for (const RLBodyDof3D& dof : body.dofs) {
            Vec3& q = frame.jointCoordinates[dof.linkIndex];
            Vec3& qd = frame.jointVelocities[dof.linkIndex];
            float* value = dof.axis == RagdollAxis3D::Twist ? &q.x : dof.axis == RagdollAxis3D::Swing1 ? &q.y : &q.z;
            float* rate = dof.axis == RagdollAxis3D::Twist ? &qd.x : dof.axis == RagdollAxis3D::Swing1 ? &qd.y : &qd.z;
            *value = std::clamp(*value + gaussian(env.rng, start.jointNoiseRadians * noise), dof.minimumRadians,
                dof.maximumRadians);
            *rate += gaussian(env.rng, start.jointVelocityNoise * noise);
        }
        frame.rootVelocity += Vec3 { gaussian(env.rng, start.rootVelocityNoise * noise),
            gaussian(env.rng, start.rootVelocityNoise * noise), 0.0f };
        frame.rootSpin += Vec3 { gaussian(env.rng, start.rootAngularVelocityNoise * noise),
            gaussian(env.rng, start.rootAngularVelocityNoise * noise),
            gaussian(env.rng, start.rootAngularVelocityNoise * noise) };
        const float yaw = startYaw;
        // O pe de apoio 2 mm acima do chao (no voo, como o clipe).
        const float lowest = rlBodyLowestFootPoint3D(body, referencePose3D(body, frame, env.origin, yaw));
        referenceLinkStates3D(body, frame, env.origin + Vec3 { 0.0f, 0.0f, std::max(0.0f, 0.002f - lowest) }, yaw, links);
    } else {
        std::vector<Vec3> coordinates = posture.jointCoordinates;
        std::vector<Vec3> velocities(coordinates.size());
        for (const RLBodyDof3D& dof : body.dofs) {
            Vec3& q = coordinates[dof.linkIndex];
            Vec3& qd = velocities[dof.linkIndex];
            const float dq = gaussian(env.rng, start.jointNoiseRadians * noise);
            const float dqd = gaussian(env.rng, start.jointVelocityNoise * noise);
            float* value = dof.axis == RagdollAxis3D::Twist ? &q.x : dof.axis == RagdollAxis3D::Swing1 ? &q.y : &q.z;
            float* rate = dof.axis == RagdollAxis3D::Twist ? &qd.x : dof.axis == RagdollAxis3D::Swing1 ? &qd.y : &qd.z;
            *value = std::clamp(*value + dq, dof.minimumRadians, dof.maximumRadians);
            *rate = dqd;
        }
        const float yaw = start.randomYaw
            ? std::uniform_real_distribution<float>(-std::numbers::pi_v<float>, std::numbers::pi_v<float>)(env.rng) : 0.0f;
        const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
        const Quaternion root = (heading * body.profile.links.front().modelOrientation * posture.rootRotation).normalized();
        const RLBodyPose3D probe = rlBodyPose3D(body.profile, env.origin, root, coordinates);
        const float lift = 0.002f - rlBodyLowestFootPoint3D(body, probe);
        const Vec3 rootPosition = env.origin + Vec3 { 0.0f, 0.0f, lift };
        Vec3 rootVelocity { gaussian(env.rng, start.rootVelocityNoise * noise),
            gaussian(env.rng, start.rootVelocityNoise * noise), 0.0f };
        if (training() && !teaching() && task.commands.startMoving > 0.0f
            && std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng) < task.commands.startMoving) {
            // Ja na velocidade pedida (no rumo do corpo): o passo vira reacao.
            const Vec3 forward = (root * body.profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 left { -forward.y, forward.x, 0.0f };
            rootVelocity += Vec3 { forward.x, forward.y, 0.0f } * env.command.x + left * env.command.y;
        }
        const Vec3 rootSpin { gaussian(env.rng, start.rootAngularVelocityNoise * noise),
            gaussian(env.rng, start.rootAngularVelocityNoise * noise),
            gaussian(env.rng, start.rootAngularVelocityNoise * noise) };
        rlBodyLinkStates3D(body.profile, rootPosition, root, rootVelocity, rootSpin, coordinates, velocities, links);
    }
    scene.physics->resetRagdollState(env.ragdoll, links);
    {
        const Vec3 forward = (links.front().orientation * body.profile.links.front().modelOrientation.conjugate())
            .rotate({ 1.0f, 0.0f, 0.0f });
        env.startHeading = std::atan2(forward.y, forward.x);
    }
    env.footContactTime = {};
    env.footAirTime = {};
    env.gaitPhase = 0.0f;

    // O mundo e o corpo deste episodio.
    const TrainingTask3D::Randomization& random = task.randomization;
    const bool varied = training() && !teaching();
    env.friction = varied ? uniform(env.rng, random.friction) : 1.0f;
    env.strength = varied ? uniform(env.rng, random.strength) : 1.0f;
    env.delayed = varied && std::uniform_real_distribution<float>(0.0f, 1.0f)(env.rng)
            < random.actionDelayProbability;
    scene.physics->setRagdollFrictionScale(env.ragdoll, env.friction);
    env.limit = training()
        ? std::max(1u, static_cast<std::uint32_t>(std::lround(static_cast<float>(maximumSteps)
              * std::uniform_real_distribution<float>(0.8f, 1.2f)(env.rng))))
        : maximumSteps;
    std::fill(env.previousAction.begin(), env.previousAction.end(), 0.0f);
    std::fill(env.action.begin(), env.action.end(), 0.0f);
    writeTargets(env, {});
    env.previousTargets = env.targets;
    scene.physics->setRagdollActiveDriveTargets(env.ragdoll, env.targets, false);
    env.firstContactLink = -1;
    env.pushTimer = uniform(env.rng, task.pushes.intervalSeconds);
    env.pushRemaining = 0.0f;
    env.pushForce = {};
    const RagdollState3D* state = scene.physics->ragdollStateView(env.ragdoll);
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        env.footStart[f] = state ? state->links[foot].position : Vec3 {};
        env.footStartOrientation[f] = state ? state->links[foot].orientation : Quaternion {};
    }
    env.imitationErrorSquares = 0.0;
    env.facingErrorSquares = 0.0;
    env.facingSteps = 0;
    if (targeting() && state && !(imitating() && reference.library)) placeFirstTarget(env, *state);
    env.episodeReturn = 0.0;
    env.comSum[0] = env.comSum[1] = 0.0;
    env.comSquares[0] = env.comSquares[1] = 0.0;
    env.footDriftMax = 0.0f;
    env.footTurnMax = 0.0f;
    env.actionRateSum = env.effortSum = env.jointSpeedSum = env.uprightSum = env.trunkTurnSquares = 0.0;
    env.poseErrorSquares = env.velocityErrorSquares = env.yawErrorSquares = env.trackingSum = 0.0;
    env.movingSteps = 0;
    if (teaching() && state) {
        std::fill(env.teacherSum.begin(), env.teacherSum.end(), 0.0f);
        settings.teacher->beginEpisode(teacherWorld(index), *state, env.rng());
    }
}

void TrainingBatch3D::Impl::accumulateTeacher(const RagdollState3D& state, std::vector<float>& sum) const {
    // O alvo do RL que da o mesmo torque: o do RL e k (alvo - q) - c q', sem
    // alvo de velocidade nem antecipacao; o do professor, o do motor mais a
    // antecipacao dele (a compensacao de gravidade inclusa).
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        const std::size_t axis = static_cast<std::size_t>(dof.axis);
        const float torque = joint.motorTorqueNewtonMeters[axis] + joint.feedforwardTorqueNewtonMeters[axis];
        const float q = joint.positionRadians[axis];
        const float target = q + (torque + dof.damping * joint.velocityRadiansPerSecond[axis]) / std::max(1.0e-3f, dof.stiffness);
        sum[d] += (target - dof.baseRadians) / std::max(1.0e-4f, dof.actionScale);
    }
}

BodyReading TrainingBatch3D::Impl::read(const RagdollState3D& state) const {
    BodyReading reading;
    const RagdollProfile3D& profile = body.profile;
    const PhysicsBodyState3D& pelvis = state.links.front();
    if (!finite(pelvis.position) || !finite(pelvis.linearVelocity) || !std::isfinite(pelvis.orientation.w)) {
        reading.valid = false;
        return reading;
    }
    reading.pelvisFrame = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = reading.pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
    const float yaw = std::atan2(forward.y, forward.x);
    reading.headingInverse = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -yaw);
    float massSum = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const float m = profile.links[i].massFraction;
        reading.com += state.links[i].position * m;
        reading.comVelocity += state.links[i].linearVelocity * m;
        massSum += m;
        if (!finite(state.links[i].position)) reading.valid = false;
    }
    reading.com = reading.com / massSum;
    reading.comVelocity = reading.comVelocity / massSum;
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        const PhysicsBodyState3D& link = state.links[foot];
        reading.footCenter[f] = link.position + link.orientation.rotate(profile.links[foot].collider.localPosition);
    }
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.otherMotion != RagdollContactMotion3D::Static || contact.linkIndex >= 32) continue;
        reading.groundMask |= 1u << contact.linkIndex;
        for (std::size_t f = 0; f < 2; ++f) {
            const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
            if (contact.linkIndex != foot) continue;
            reading.footContact[f] = true;
            reading.footForce[f] += contact.normalImpulseNewtonSeconds / FixedStep;
        }
    }
    return reading;
}

void TrainingBatch3D::Impl::writeObservation(const Environment& env, const RagdollState3D& state,
    const BodyReading& reading, float* actor, float* criticOut, bool noise) {
    const RagdollProfile3D& profile = body.profile;
    const PhysicsBodyState3D& pelvis = state.links.front();
    std::mt19937_64& rng = const_cast<Environment&>(env).rng;
    const float k = noise ? task.randomization.observationNoise : 0.0f;
    const auto n = [&](float sigma) { return k > 0.0f ? gaussian(rng, sigma * k) : 0.0f; };
    const auto put3 = [](float*& out, Vec3 v) {
        *out++ = v.x;
        *out++ = v.y;
        *out++ = v.z;
    };
    const auto noisy3 = [&](Vec3 v, float sigma) { return Vec3 { v.x + n(sigma), v.y + n(sigma), v.z + n(sigma) }; };

    float* out = actor;
    *out++ = (pelvis.position.z - body.standingPelvisHeight) / 0.3f + n(0.005f);
    put3(out, noisy3(reading.pelvisFrame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f));
    put3(out, noisy3(reading.headingInverse.rotate(pelvis.linearVelocity) * 0.5f, 0.02f));
    put3(out, noisy3(reading.pelvisFrame.conjugate().rotate(pelvis.angularVelocity) * 0.25f, 0.02f));
    for (const RLBodyDof3D& dof : body.dofs) {
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        *out++ = joint.positionRadians[static_cast<std::size_t>(dof.axis)] - dof.baseRadians + n(0.01f);
    }
    for (const RLBodyDof3D& dof : body.dofs) {
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        *out++ = joint.velocityRadiansPerSecond[static_cast<std::size_t>(dof.axis)] * 0.1f + n(0.015f);
    }
    for (const std::uint32_t link : keyBodies)
        put3(out, noisy3(reading.headingInverse.rotate(state.links[link].position - pelvis.position), 0.005f));
    for (const std::uint32_t foot : { body.leftFoot, body.rightFoot }) {
        const Quaternion frame = state.links[foot].orientation * profile.links[foot].modelOrientation.conjugate();
        put3(out, noisy3(frame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f));
    }
    put3(out, noisy3(reading.headingInverse.rotate(reading.com - pelvis.position), 0.005f));
    put3(out, noisy3(reading.headingInverse.rotate(reading.comVelocity) * 0.5f, 0.02f));
    const Vec3 support = (reading.footCenter[0] + reading.footCenter[1]) * 0.5f;
    const Vec3 offset = reading.headingInverse.rotate(Vec3 { reading.com.x - support.x, reading.com.y - support.y, 0.0f });
    *out++ = offset.x * 5.0f + n(0.02f);
    *out++ = offset.y * 5.0f + n(0.02f);
    const float weight = profile.totalMassKg * env.massScale * Gravity;
    for (std::size_t f = 0; f < 2; ++f) *out++ = reading.footContact[f] ? 1.0f : 0.0f;
    for (std::size_t f = 0; f < 2; ++f) *out++ = std::min(3.0f, reading.footForce[f] / weight);
    for (const float a : env.action) *out++ = a;
    for (std::size_t p = 1; p < body.postures.size(); ++p) *out++ = env.posture == p ? 1.0f : 0.0f;
    *out++ = env.command.x / 3.0f;
    *out++ = env.command.y / 3.0f;
    *out++ = env.command.z / 2.0f;
    const bool walking = env.command.x != 0.0f || env.command.y != 0.0f || env.command.z != 0.0f;
    *out++ = walking ? std::sin(2.0f * std::numbers::pi_v<float> * env.gaitPhase) : 0.0f;
    *out++ = walking ? std::cos(2.0f * std::numbers::pi_v<float> * env.gaitPhase) : 0.0f;
    if (imitating()) {
        // A referencia no fim desta decisao: o trecho, a fase e as juntas.
        const ReferenceCursor3D next = advanceReference3D(reference, env.cursor, task.actionRepeat);
        const bool loop = reference.segments[next.segment].loops;
        const float progress = referenceProgress3D(reference, next);
        *out++ = loop ? 0.0f : 1.0f;
        *out++ = loop ? 0.0f : progress;
        *out++ = loop ? std::sin(2.0f * std::numbers::pi_v<float> * progress) : 0.0f;
        *out++ = loop ? std::cos(2.0f * std::numbers::pi_v<float> * progress) : 0.0f;
        const ReferenceFrame3D& frame = referenceFrame3D(reference, next);
        for (std::size_t d = 0; d < body.dofs.size(); ++d) *out++ = frame.dofRadians[d] - body.dofs[d].baseRadians;
    }
    if (targeting()) {
        const Vec3 to = reading.headingInverse.rotate(Vec3 { env.target.x - pelvis.position.x,
            env.target.y - pelvis.position.y, 0.0f });
        const float distance = std::hypot(to.x, to.y);
        *out++ = distance > 1.0e-4f ? to.x / distance : 1.0f;
        *out++ = distance > 1.0e-4f ? to.y / distance : 0.0f;
        *out++ = std::min(distance, 20.0f) / 10.0f;
        for (std::uint32_t i = 0; i < task.target.lookahead; ++i) {
            const Vec3 next = reading.headingInverse.rotate(Vec3 { env.upcoming[i].x - pelvis.position.x,
                env.upcoming[i].y - pelvis.position.y, 0.0f });
            const float far = std::hypot(next.x, next.y);
            *out++ = far > 1.0e-4f ? next.x / far : 1.0f;
            *out++ = far > 1.0e-4f ? next.y / far : 0.0f;
            *out++ = std::min(far, 30.0f) / 10.0f;
        }
    }
    if (imitating() && reference.library) {
        for (std::size_t c = 0; c < reference.segments.size(); ++c) *out++ = env.cursor.segment == c ? 1.0f : 0.0f;
        *out++ = env.turnRate / 2.0f;
        if (task.target.facing) {
            // Para onde olhar, em relacao ao rumo do que fica virado para o
            // comando (zero sem ele), e se ele vale neste episodio.
            const float error = env.facingActive ? env.facingCommand - facingLinkHeading(state) : 0.0f;
            *out++ = std::sin(error);
            *out++ = std::cos(error);
            *out++ = env.facingActive ? 1.0f : 0.0f;
        }
    }

    if (criticOut == nullptr) return;
    // O critico: a observacao sem ruido mais o mundo que a politica nao ve.
    if (k > 0.0f) {
        writeObservation(env, state, reading, criticOut, nullptr, false);
    } else {
        std::copy(actor, actor + observationSize(), criticOut);
    }
    float* extra = criticOut + observationSize();
    *extra++ = env.friction;
    *extra++ = env.massScale;
    *extra++ = env.strength;
    *extra++ = env.delayed ? 1.0f : 0.0f;
    const Vec3 push = env.pushRemaining > 0.0f ? reading.headingInverse.rotate(env.pushForce) / 200.0f : Vec3 {};
    put3(extra, push);
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        *extra++ = (reading.groundMask >> i) & 1u ? 1.0f : 0.0f;
}

float TrainingBatch3D::Impl::reward(Environment& env, const RagdollState3D& state, const BodyReading& reading,
    bool fell, float* componentsOut) {
    const TrainingRewardWeights3D& w = task.reward;
    const RagdollProfile3D& profile = body.profile;
    const RLPosture3D& posture = body.postures[env.posture];
    // Andando ou parado: as parcelas de parado (quieto, centro de massa sobre
    // os pes, pe saindo do lugar) so valem com comando zero.
    const bool moving = env.command.x != 0.0f || env.command.y != 0.0f || env.command.z != 0.0f;
    const float standing = moving ? 0.0f : 1.0f;

    // Pelve, peito e cabeca na postura pedida, no referencial do rumo: o
    // angulo entre o "para cima" de cada um e o da postura.
    const auto deviation = [&](std::uint32_t link, Vec3 base) {
        const Quaternion frame = state.links[link].orientation * profile.links[link].modelOrientation.conjugate();
        const Vec3 up = reading.headingInverse.rotate(frame.rotate({ 0.0f, 0.0f, 1.0f }));
        return std::acos(std::clamp(dot(up, base), -1.0f, 1.0f));
    };
    const float pelvisTilt = deviation(0, posture.pelvisUp);
    const float chestTilt = deviation(body.upperChest, posture.chestUp);
    const float headTilt = deviation(body.head, posture.headUp);
    const float upright = 0.4f * std::exp(-pelvisTilt * pelvisTilt / 0.06f)
        + 0.3f * std::exp(-chestTilt * chestTilt / 0.06f) + 0.3f * std::exp(-headTilt * headTilt / 0.06f);
    const float headError = (state.links[body.head].position.z - posture.standingHeadHeight) / 0.08f;
    const float headHeight = std::exp(-headError * headError);
    const Vec3 support = (reading.footCenter[0] + reading.footCenter[1]) * 0.5f;
    const float dx = reading.com.x - support.x, dy = reading.com.y - support.y;
    const float comOverSupport = std::exp(-(dx * dx + dy * dy) / (0.06f * 0.06f));
    const float comSpeed2 = reading.comVelocity.lengthSquared();
    const float still = std::exp(-comSpeed2 / 0.04f);
    // O tronco girando: a media dos quadrados da velocidade angular da pelve e
    // do peito (rad/s), com teto de 2 rad/s (numa queda ja paga o "fall").
    // Andando, so o balanco para os lados e para frente e tras (o giro em torno
    // da vertical e o comandado, e a pelve gira um pouco a cada passo).
    const auto turn2 = [&](Vec3 w) { return moving ? w.x * w.x + w.y * w.y : w.lengthSquared(); };
    const float trunkTurn2
        = 0.5f * (turn2(state.links[0].angularVelocity) + turn2(state.links[body.upperChest].angularVelocity));
    const float trunkTurn = std::min(trunkTurn2, 4.0f);
    // Andar: a velocidade da pelve no referencial do rumo e o giro contra o pedido.
    const Vec3 velocity = reading.headingInverse.rotate(state.links[0].linearVelocity);
    const float velocityError2
        = (velocity.x - env.command.x) * (velocity.x - env.command.x) + (velocity.y - env.command.y) * (velocity.y - env.command.y);
    const float yawError = state.links[0].angularVelocity.z - env.command.z;
    const float velocityTracking = std::exp(-velocityError2 / 0.25f);
    const float yawTracking = std::exp(-yawError * yawError / 0.25f);
    // O pe no ar: num apoio simples, o menor tempo no modo de agora dos dois
    // pes (o de apoio no chao, o outro no ar), ate 0,4 s; so andando (o passo
    // de verdade, dos humanoides do Isaac Lab).
    const float stepSeconds = static_cast<float>(task.actionRepeat) * FixedStep;
    for (std::size_t f = 0; f < 2; ++f) {
        if (reading.footContact[f]) {
            env.footContactTime[f] += stepSeconds;
            env.footAirTime[f] = 0.0f;
        } else {
            env.footAirTime[f] += stepSeconds;
            env.footContactTime[f] = 0.0f;
        }
    }
    float airTime = 0.0f;
    if (reading.footContact[0] != reading.footContact[1] && std::hypot(env.command.x, env.command.y) > 0.1f) {
        const auto mode = [&](std::size_t f) { return reading.footContact[f] ? env.footContactTime[f] : env.footAirTime[f]; };
        airTime = std::min(0.4f, std::min(mode(0), mode(1)));
    }
    // O relogio de passada: o esquerdo apoia na metade de seno positivo da fase
    // e o direito na outra; perto da troca, apoio duplo devagar e voo rapido
    // (a faixa d muda de sinal em flightSpeed). O pe que devia estar no ar
    // paga estando no ar e desconta no chao, e sobe ate a altura do passo.
    float gaitContact = 0.0f, swingHeight = 0.0f;
    if (moving) {
        const TrainingTask3D::Gait& gait = task.gait;
        const float speed = std::hypot(env.command.x, env.command.y);
        const float band = gait.doubleSupport * (1.0f - speed / gait.flightSpeed);
        const float phase = std::sin(2.0f * std::numbers::pi_v<float> * env.gaitPhase);
        const std::array<bool, 2> stance { phase >= -band, phase <= band };
        float swinging = 0.0f;
        for (std::size_t f = 0; f < 2; ++f) {
            // So o passo paga: o pe de balanco no ar +0,5 (no chao -0,5) e o de
            // apoio no chao 0 (no ar -0,5). Parado, com os dois pes plantados,
            // nunca passa de 0 (no apoio duplo, o "acerto" dos dois pagava +1).
            if (stance[f]) gaitContact += reading.footContact[f] ? 0.0f : -0.5f;
            else gaitContact += reading.footContact[f] ? -0.5f : 0.5f;
            if (stance[f]) continue;
            const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
            const float height = state.links[foot].position.z - env.footStart[f].z;
            swingHeight += std::clamp(height / gait.swingHeight, 0.0f, 1.0f);
            swinging += 1.0f;
        }
        if (swinging > 0.0f) swingHeight /= swinging;
    }
    // A referencia de passada: o balanco de cada perna (0 no apoio e no apoio
    // duplo, 1 no meio do voo) puxa as juntas pedidas da postura ao alvo.
    float gaitReference = 0.0f;
    if (moving && task.gait.referenceHipFlexion > 0.0f && !referenceDofs.empty()) {
        const TrainingTask3D::Gait& gait = task.gait;
        const float speed = std::hypot(env.command.x, env.command.y);
        const float fade = std::clamp((gait.referenceFade.maximum - speed)
                / (gait.referenceFade.maximum - gait.referenceFade.minimum), 0.0f, 1.0f);
        const float band = std::max(0.0f, gait.doubleSupport * (1.0f - speed / gait.flightSpeed));
        const float phase = std::sin(2.0f * std::numbers::pi_v<float> * env.gaitPhase);
        const float open = std::max(1.0e-3f, 1.0f - band);
        const std::array<float, 2> swing { std::max(0.0f, -phase - band) / open, std::max(0.0f, phase - band) / open };
        float distance2 = 0.0f;
        for (const ReferenceDof& reference : referenceDofs) {
            const RLBodyDof3D& dof = body.dofs[reference.dof];
            const float q = state.joints[dof.linkIndex].positionRadians[static_cast<std::size_t>(dof.axis)];
            const float target
                = posture.dofRadians[reference.dof] + reference.scale * gait.referenceHipFlexion * swing[reference.leg];
            distance2 += (q - target) * (q - target);
        }
        const float distance = std::sqrt(distance2);
        gaitReference = fade * (std::exp(-2.0f * distance) - 0.2f * std::min(distance, 0.5f));
    }
    // Medido contra ficar parado (relativeToStanding): 0 parado, 1 seguindo o
    // pedido. Sem pedido de giro, o giro segue em sino (nao girar).
    const float linear2 = env.command.x * env.command.x + env.command.y * env.command.y;
    const bool relative = moving && task.commands.relativeToStanding;
    const auto overStanding = [](float tracking, float commanded2) {
        const float standing = std::exp(-commanded2 / 0.25f);
        return std::clamp((tracking - standing) / std::max(1.0e-3f, 1.0f - standing), 0.0f, 1.0f);
    };
    const float linearScore = relative && linear2 > 0.0f ? overStanding(velocityTracking, linear2) : velocityTracking;
    const float yawScore
        = relative && env.command.z != 0.0f ? overStanding(yawTracking, env.command.z * env.command.z) : yawTracking;
    // Com pedido de andar, "estar bem" so paga andando: vivo, postura, cabeca,
    // pose (e giro, no sino puro) multiplicados pelo acompanhamento.
    float gate = 1.0f;
    if (moving && task.commands.gateByTracking)
        gate = relative ? std::min(linear2 > 0.0f ? linearScore : 1.0f, env.command.z != 0.0f ? yawScore : 1.0f)
                        : velocityTracking;

    float slip = 0.0f, drift = 0.0f;
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
        const PhysicsBodyState3D& link = state.links[foot];
        // A sola escorregando: o centro do pe mais o giro em torno da vertical
        // (um pe que pivota no lugar quase nao move o centro; 0,1 m e meia sola).
        if (reading.footContact[f])
            slip += std::sqrt(link.linearVelocity.x * link.linearVelocity.x + link.linearVelocity.y * link.linearVelocity.y)
                + 0.1f * std::abs(link.angularVelocity.z);
        // O quanto o pe girou no chao desde o inicio (o giro em torno da vertical).
        const Quaternion turned = link.orientation * env.footStartOrientation[f].conjugate();
        const float twist = 2.0f * std::atan2(std::abs(turned.z), std::abs(turned.w));
        env.footTurnMax = std::max(env.footTurnMax, twist);
        const float mx = link.position.x - env.footStart[f].x, my = link.position.y - env.footStart[f].y;
        const float moved = std::sqrt(mx * mx + my * my);
        drift += moved;
        env.footDriftMax = std::max(env.footDriftMax, moved);
    }

    float effort = 0.0f, actionRate = 0.0f, limit = 0.0f, pose = 0.0f, speed = 0.0f, poseError = 0.0f;
    float upperError = 0.0f, upperCount = 0.0f;
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        const std::size_t axis = static_cast<std::size_t>(dof.axis);
        const float cap = std::max(1.0f, dof.maximumTorque * env.strength);
        const float torque = joint.motorTorqueNewtonMeters[axis] / cap;
        effort += torque * torque;
        const float da = env.action[d] - env.previousAction[d];
        actionRate += da * da;
        const float range = std::max(0.01f, dof.maximumRadians - dof.minimumRadians);
        const float q = joint.positionRadians[axis];
        const float x = (q - dof.minimumRadians) / range;
        limit += std::max(0.0f, 0.05f - x) / 0.05f + std::max(0.0f, x - 0.95f) / 0.05f;
        const float fromPosture = 2.0f * (q - posture.dofRadians[d]) / range;
        pose += fromPosture * fromPosture;
        poseError += (q - posture.dofRadians[d]) * (q - posture.dofRadians[d]);
        if (upperDof[d]) {
            upperError += (q - posture.dofRadians[d]) * (q - posture.dofRadians[d]);
            upperCount += 1.0f;
        }
        speed += std::abs(joint.velocityRadiansPerSecond[axis]);
    }
    const float dofCount = static_cast<float>(std::max<std::size_t>(1, body.dofs.size()));
    effort /= dofCount;
    actionRate /= dofCount;
    limit /= dofCount;
    pose /= dofCount;
    speed /= dofCount;
    poseError /= dofCount;
    upperError /= std::max(1.0f, upperCount);
    // A postura pedida: parado, o corpo todo (sigma ~10 graus); andando, so o
    // tronco e os bracos, mais folgados (sigma ~18 graus: os bracos balancam).
    const float poseMatch = moving ? std::exp(-upperError / 0.1f) : std::exp(-poseError / 0.03f);

    // A imitacao: o corpo contra a referencia no mesmo instante (o fim desta
    // decisao), tudo no referencial do rumo de cada um.
    float imitationPose = 0.0f, imitationVelocity = 0.0f, imitationKeyBodies = 0.0f, imitationRoot = 0.0f;
    if (imitating()) {
        const TrainingTask3D::Imitation& m = task.imitation;
        const ReferenceFrame3D& frame = referenceFrame3D(reference, env.cursor);
        float pose2 = 0.0f, velocity2 = 0.0f;
        for (std::size_t d = 0; d < body.dofs.size(); ++d) {
            const RLBodyDof3D& dof = body.dofs[d];
            const RagdollJointState3D& joint = state.joints[dof.linkIndex];
            const std::size_t axis = static_cast<std::size_t>(dof.axis);
            const float e = joint.positionRadians[axis] - frame.dofRadians[d];
            const float ev = joint.velocityRadiansPerSecond[axis] - frame.dofVelocities[d];
            pose2 += e * e;
            velocity2 += ev * ev;
        }
        env.imitationErrorSquares += pose2 / dofCount;
        // A inclinacao da pelve entra na pose (a pelve nao tem junta).
        const Vec3 up = reading.headingInverse.rotate(reading.pelvisFrame.rotate({ 0.0f, 0.0f, 1.0f }));
        const float tilt = std::acos(std::clamp(dot(up, frame.pelvisUp), -1.0f, 1.0f));
        pose2 += tilt * tilt;
        const PhysicsBodyState3D& pelvis = state.links.front();
        float key2 = 0.0f;
        for (std::size_t k = 0; k < KeyBodyCount; ++k) {
            const Vec3 p = reading.headingInverse.rotate(state.links[keyBodies[k]].position - pelvis.position);
            key2 += (p - frame.keyBodies[k]).lengthSquared();
        }
        const float height = pelvis.position.z - frame.rootHeight;
        const Vec3 rootVelocity = reading.headingInverse.rotate(pelvis.linearVelocity);
        imitationPose = std::exp(-m.poseScale * pose2);
        imitationVelocity = std::exp(-m.velocityScale * velocity2);
        imitationKeyBodies = std::exp(-m.keyBodyScale * key2);
        imitationRoot = std::exp(-m.rootHeightScale * height * height
            - m.rootVelocityScale * (rootVelocity - frame.rootVelocity).lengthSquared());
    }
    // O alvo: a velocidade do centro de massa em direcao a ele (linear: correr
    // mais rapido paga mais) e, chegando, a velocidade media do trecho; o
    // proximo nasce na hora.
    float targetSpeed = 0.0f, targetReached = 0.0f;
    if (targeting()) {
        const TrainingTask3D::Target& t = task.target;
        const Vec3 to { env.target.x - reading.com.x, env.target.y - reading.com.y, 0.0f };
        const float distance = to.length();
        env.targetClosest = std::min(env.targetClosest, distance);
        if (distance > 1.0e-4f) {
            const float toward = (reading.comVelocity.x * to.x + reading.comVelocity.y * to.y) / distance;
            targetSpeed = std::clamp(toward / t.referenceSpeed, 0.0f, t.speedCap);
        }
        if (distance < t.radius) {
            const float now = static_cast<float>(env.played) * stepSeconds;
            const float covered = (env.target - env.targetFrom).length();
            const float speed = covered / std::max(stepSeconds, now - env.targetSince);
            targetReached = std::min(speed / t.referenceSpeed, t.speedCap);
            ++env.targetsReached;
            for (std::size_t i = env.reachedTargets.size() - 1; i > 0; --i) env.reachedTargets[i] = env.reachedTargets[i - 1];
            env.reachedTargets[0] = env.target;
            env.reachedCount = std::min<std::uint32_t>(env.reachedCount + 1, static_cast<std::uint32_t>(env.reachedTargets.size()));
            env.targetSpeedSum += speed;
            if (env.firstTargetSeconds < 0.0f) env.firstTargetSeconds = now;
            env.targetSince = now;
            if (imitating() && reference.library) {
                // O proximo em volta, a partir de onde ele esta e para onde olha.
                const Vec3 forward = reading.pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
                const float facing = env.facingActive ? env.facingCommand : std::atan2(forward.y, forward.x);
                placeLibraryTarget(env, reading.com, facing, env.targetCount);
            } else {
                placeNextTarget(env);
            }
        }
    }
    // O giro: o do corpo contra o pedido (o do clipe agora mais o que alinha a
    // direcao em que o clipe anda com o alvo, da decisao anterior).
    // Segurar o rumo pedido: o link do comando contra ele, em sino.
    float facingHold = 0.0f;
    if (env.facingActive) {
        float error = env.facingCommand - facingLinkHeading(state);
        error = std::atan2(std::sin(error), std::cos(error));
        const float sigma = task.target.facingSigma;
        facingHold = std::exp(-error * error / (sigma * sigma));
        env.facingErrorSquares += error * error;
        ++env.facingSteps;
    }
    float targetTurn = 0.0f;
    if (targeting() && imitating()) {
        const float wanted = referenceFrame3D(reference, env.cursor).rootSpin.z + env.turnRate;
        const float error = state.links.front().angularVelocity.z - wanted;
        const float sigma = task.target.turnSigma;
        targetTurn = std::exp(-error * error / (sigma * sigma));
    }

    const std::array<float, RewardCount> values {
        w.alive * gate,
        w.upright * upright * gate,
        w.headHeight * headHeight * gate,
        w.comOverSupport * comOverSupport * standing,
        w.still * still * standing,
        -w.trunkTurn * trunkTurn,
        -w.footSlip * slip,
        -w.footDrift * drift * standing,
        -w.effort * effort,
        -w.actionRate * actionRate,
        -w.jointLimit * limit,
        -w.posePrior * pose,
        w.poseMatch * poseMatch * gate,
        w.velocity * linearScore,
        // (Medido contra parado: sem giro pedido, o "nao girar" so paga andando.)
        w.yawRate * yawScore * (relative && env.command.z != 0.0f ? 1.0f : gate),
        w.feetAirTime * airTime,
        w.gaitContact * gaitContact,
        w.swingHeight * swingHeight,
        w.gaitReference * gaitReference,
        w.imitationPose * imitationPose,
        w.imitationVelocity * imitationVelocity,
        w.imitationKeyBodies * imitationKeyBodies,
        w.imitationRoot * imitationRoot,
        w.targetSpeed * targetSpeed,
        w.targetReached * targetReached,
        w.targetTurn * targetTurn,
        w.facingHold * facingHold,
        fell ? -w.fall : 0.0f,
    };
    // A queda e a ultima parcela; o resto, com positiveOnly, nunca abaixo de 0.
    float living = 0.0f;
    for (std::size_t i = 0; i < RewardCount; ++i) {
        componentsOut[i] = values[i];
        if (i + 1 < RewardCount) living += values[i];
    }
    const float total = (w.positiveOnly ? std::max(0.0f, living) : living) + values[RewardCount - 1];

    // O resumo do episodio.
    env.comSum[0] += reading.com.x;
    env.comSum[1] += reading.com.y;
    env.comSquares[0] += static_cast<double>(reading.com.x) * reading.com.x;
    env.comSquares[1] += static_cast<double>(reading.com.y) * reading.com.y;
    env.actionRateSum += actionRate;
    env.effortSum += effort;
    env.jointSpeedSum += speed;
    env.uprightSum += upright;
    env.trunkTurnSquares += trunkTurn2;
    env.poseErrorSquares += poseError;
    env.velocityErrorSquares += velocityError2;
    env.yawErrorSquares += yawError * yawError;
    if (moving) {
        env.trackingSum += velocityTracking;
        ++env.movingSteps;
    }
    return total;
}

void TrainingBatch3D::Impl::finishEpisode(Scene& scene, std::uint32_t index, TrainingEndReason3D reason,
    const BodyReading& reading) {
    const Environment& env = environments[index];
    TrainingEpisode3D episode;
    episode.environment = index;
    episode.reason = reason;
    episode.firstContactLink = env.firstContactLink;
    episode.steps = env.played;
    episode.seconds = static_cast<float>(env.played) * static_cast<float>(task.actionRepeat) * FixedStep;
    episode.episodeReturn = static_cast<float>(env.episodeReturn);
    const double n = std::max(1u, env.played);
    double variance = 0.0;
    for (std::size_t a = 0; a < 2; ++a) {
        const double mean = env.comSum[a] / n;
        variance += std::max(0.0, env.comSquares[a] / n - mean * mean);
    }
    episode.comSwayRms = static_cast<float>(std::sqrt(variance));
    episode.footDriftMax = env.footDriftMax;
    episode.footTurnMax = env.footTurnMax;
    episode.meanActionRate = static_cast<float>(env.actionRateSum / n);
    episode.meanEffort = static_cast<float>(env.effortSum / n);
    episode.meanJointSpeed = static_cast<float>(env.jointSpeedSum / n);
    episode.meanUpright = static_cast<float>(env.uprightSum / n);
    episode.trunkTurnRms = static_cast<float>(std::sqrt(env.trunkTurnSquares / n));
    episode.poseErrorRms = static_cast<float>(std::sqrt(env.poseErrorSquares / n));
    episode.velocityErrorRms = static_cast<float>(std::sqrt(env.velocityErrorSquares / n));
    episode.yawRateErrorRms = static_cast<float>(std::sqrt(env.yawErrorSquares / n));
    episode.velocityTracking = env.movingSteps > 0 ? static_cast<float>(env.trackingSum / env.movingSteps) : -1.0f;
    if (reading.valid) {
        const Vec3 moved { reading.com.x - env.origin.x, reading.com.y - env.origin.y, 0.0f };
        episode.distance = moved.length();
        episode.lateralDrift = std::abs(-std::sin(env.startHeading) * moved.x + std::cos(env.startHeading) * moved.y);
    }
    episode.targetsReached = env.targetsReached;
    episode.firstTargetSeconds = env.firstTargetSeconds;
    episode.meanTargetSpeed = env.targetsReached > 0 ? static_cast<float>(env.targetSpeedSum / env.targetsReached) : 0.0f;
    episode.imitationErrorRms = imitating() ? static_cast<float>(std::sqrt(env.imitationErrorSquares / n)) : -1.0f;
    episode.facingErrorRms =
        env.facingSteps > 0 ? static_cast<float>(std::sqrt(env.facingErrorSquares / env.facingSteps)) : -1.0f;
    if (reason == TrainingEndReason3D::FallContact || reason == TrainingEndReason3D::FallHeight) {
        // O lado da queda: para onde o centro de massa ia, no referencial do corpo.
        const Vec3 v = reading.valid ? reading.headingInverse.rotate(reading.comVelocity) : Vec3 {};
        episode.fallDirection = std::abs(v.x) >= std::abs(v.y) ? (v.x >= 0.0f ? 0 : 1) : (v.y >= 0.0f ? 2 : 3);
    }
    scene.finished.push_back(episode);
}

void TrainingBatch3D::Impl::stepScene(Scene& scene, std::span<const float> actions) {
    PhysicsScene3D& physics = *scene.physics;
    const std::size_t A = actionSize();
    scene.finished.clear();
    const bool teacher = teaching();
    for (const std::uint32_t index : scene.environments) {
        Environment& env = environments[index];
        std::swap(env.previousAction, env.action);
        // Com professor, os alvos sao os dele (a cada passo de fisica).
        if (teacher) {
            std::fill(env.teacherSum.begin(), env.teacherSum.end(), 0.0f);
            continue;
        }
        for (std::size_t d = 0; d < A; ++d)
            env.action[d] = std::clamp(actions[index * A + d], -RLActionLimit, RLActionLimit);
        std::swap(env.previousTargets, env.targets);
        writeTargets(env, env.action);
        physics.setRagdollActiveDriveTargets(env.ragdoll, env.delayed ? env.previousTargets : env.targets, false);
    }

    const bool pushes = task.pushes.enabled && !pushLinks.empty();
    for (std::uint32_t sub = 0; sub < task.actionRepeat; ++sub) {
        for (const std::uint32_t index : scene.environments) {
            Environment& env = environments[index];
            // Atraso de um passo fisico: o comando novo entra no segundo.
            if (sub == 1 && env.delayed) physics.setRagdollActiveDriveTargets(env.ragdoll, env.targets, false);
            if (env.pushRemaining > 0.0f) {
                physics.applyRagdollLinkForce(env.ragdoll, env.pushLink, env.pushForce, {});
                env.pushRemaining -= FixedStep;
                continue;
            }
            if (!pushes) continue;
            if ((env.pushTimer -= FixedStep) <= 0.0f) {
                const float angle =
                    std::uniform_real_distribution<float>(-std::numbers::pi_v<float>, std::numbers::pi_v<float>)(env.rng);
                const float force = uniform(env.rng, task.pushes.forceNewtons);
                env.pushForce = { std::cos(angle) * force, std::sin(angle) * force, 0.0f };
                env.pushLink = pushLinks[std::uniform_int_distribution<std::size_t>(0, pushLinks.size() - 1)(env.rng)];
                env.pushRemaining = uniform(env.rng, task.pushes.durationSeconds);
                env.pushTimer = uniform(env.rng, task.pushes.intervalSeconds);
            }
        }
        if (teacher)
            for (const std::uint32_t index : scene.environments) {
                const Environment& env = environments[index];
                if (const RagdollState3D* state = physics.ragdollStateView(env.ragdoll))
                    settings.teacher->control(teacherWorld(index), *state, FixedStep);
            }
        physics.simulate(FixedStep);
        if (teacher)
            for (const std::uint32_t index : scene.environments) {
                Environment& env = environments[index];
                if (const RagdollState3D* state = physics.ragdollStateView(env.ragdoll))
                    accumulateTeacher(*state, env.teacherSum);
            }
        // Queda por contato em qualquer subpasso: um link que nao e apoio no chao.
        if (!task.termination.nonSupportContact) continue;
        for (const std::uint32_t index : scene.environments) {
            Environment& env = environments[index];
            if (env.firstContactLink >= 0) continue;
            const RagdollState3D* state = physics.ragdollStateView(env.ragdoll);
            if (!state) continue;
            for (const RagdollContactPoint3D& contact : state->contacts) {
                if (contact.otherMotion != RagdollContactMotion3D::Static) continue;
                if (contact.linkIndex < body.supportLink.size() && body.supportLink[contact.linkIndex]) continue;
                env.firstContactLink = static_cast<int>(contact.linkIndex);
                break;
            }
        }
    }

    const std::size_t O = observationSize();
    const std::size_t C = criticSize();
    if (teacher)
        for (const std::uint32_t index : scene.environments) {
            Environment& env = environments[index];
            const float scale = 1.0f / static_cast<float>(std::max(1u, task.actionRepeat));
            for (std::size_t d = 0; d < A; ++d) {
                env.action[d] = std::clamp(env.teacherSum[d] * scale, -RLActionLimit, RLActionLimit);
                teacherLabels[index * A + d] = env.action[d];
            }
        }
    for (const std::uint32_t index : scene.environments) {
        Environment& env = environments[index];
        ++env.steps;
        ++env.played;
        // A referencia andou junto com a fisica: a recompensa compara com o
        // quadro de agora.
        if (imitating()) env.cursor = advanceReference3D(reference, env.cursor, task.actionRepeat);
        const RagdollState3D* state = physics.ragdollStateView(env.ragdoll);
        float* actor = observations.data() + index * O;
        float* criticOut = critic.data() + index * C;
        float* parts = components.data() + index * RewardCount;
        BodyReading reading;
        if (state) reading = read(*state);
        else reading.valid = false;

        TrainingEndReason3D reason = TrainingEndReason3D::None;
        if (!reading.valid) {
            reason = TrainingEndReason3D::Numeric;
        } else {
            const PhysicsBodyState3D& pelvis = state->links.front();
            const float head = state->links[body.head].position.z;
            if (env.firstContactLink >= 0) reason = TrainingEndReason3D::FallContact;
            else if (head < body.postures[env.posture].standingHeadHeight * task.termination.minimumHeadHeightFraction
                || pelvis.position.z
                    < body.postures[env.posture].standingPelvisHeight * task.termination.minimumPelvisHeightFraction)
                reason = TrainingEndReason3D::FallHeight;
            else if (env.steps >= env.limit) reason = TrainingEndReason3D::TimeLimit;
            else {
                const float dx = pelvis.position.x - env.origin.x, dy = pelvis.position.y - env.origin.y;
                if (dx * dx + dy * dy > task.termination.maximumDistanceFromStart * task.termination.maximumDistanceFromStart)
                    reason = TrainingEndReason3D::Boundary;
                // Passou do alvo (ou correu para longe dele): o episodio acaba
                // (no visualizador, ele recomeca logo).
                else if (targeting() && task.target.missMeters > 0.0f) {
                    const float tx = env.target.x - reading.com.x, ty = env.target.y - reading.com.y;
                    if (std::sqrt(tx * tx + ty * ty) > env.targetClosest + task.target.missMeters)
                        reason = TrainingEndReason3D::TargetMissed;
                }
            }
        }
        const bool fell = reason == TrainingEndReason3D::FallContact || reason == TrainingEndReason3D::FallHeight;
        if (reading.valid) {
            rewards[index] = reward(env, *state, reading, fell, parts);
            // O comando da proxima decisao (a recompensa usou o que valia neste
            // passo). Quando ele muda, a referencia dos pes recomeca: o "pe
            // saindo do lugar" de quem para depois de andar e de agora.
            const Vec3 previous = env.command;
            // A fase anda com o comando deste passo (a cadencia sobe com a
            // velocidade) e volta a 0 quando o proximo pede parado.
            if (teacher) {
                // O que o professor tenta, no rumo do corpo, e a fase da passada dele.
                const Vec3 wanted = settings.teacher->command(index);
                const Vec3 local = reading.headingInverse.rotate({ wanted.x, wanted.y, 0.0f });
                env.command = { local.x, local.y, wanted.z };
                if (std::hypot(env.command.x, env.command.y) < 0.3f) env.command.x = env.command.y = 0.0f;
                if (std::abs(env.command.z) < 0.2f) env.command.z = 0.0f;
                env.gaitPhase = std::max(0.0f, settings.teacher->gaitPhase(index));
            } else {
                const float speed = std::hypot(previous.x, previous.y);
                const float frequency = task.gait.frequency.minimum + task.gait.frequency.maximum * speed;
                env.gaitPhase = std::fmod(env.gaitPhase + frequency * static_cast<float>(task.actionRepeat) * FixedStep, 1.0f);
            }
            updateCommand(env, false);
            if (env.command.x == 0.0f && env.command.y == 0.0f && env.command.z == 0.0f) env.gaitPhase = 0.0f;
            if (env.command.x != previous.x || env.command.y != previous.y || env.command.z != previous.z)
                for (std::size_t f = 0; f < 2; ++f) {
                    const std::uint32_t foot = f == 0 ? body.leftFoot : body.rightFoot;
                    env.footStart[f] = state->links[foot].position;
                    env.footStartOrientation[f] = state->links[foot].orientation;
                }
            // O planejador (biblioteca) ou so o giro pedido, para a proxima decisao.
            if (imitating() && targeting()) {
                const Vec3 forward = reading.pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
                const float facing = std::atan2(forward.y, forward.x);
                const float decision = static_cast<float>(task.actionRepeat) * FixedStep;
                env.clipSince += decision;
                env.facingHeading = facingLinkHeading(*state);
                if (env.facingActive && task.target.facingDriftRadians > 0.0f && training()) {
                    if ((env.facingTimer -= decision) <= 0.0f) {
                        env.facingTimer = std::uniform_real_distribution<float>(2.0f, 4.0f)(env.rng);
                        env.facingRate = std::uniform_real_distribution<float>(-task.target.facingDriftRadians,
                            task.target.facingDriftRadians)(env.rng);
                    }
                    env.facingCommand += env.facingRate * decision;
                }
                if (reference.library) plan(env, reading.com, facing, false);
                else env.turnRate = wantedTurn(env, reading.com, facing);
            }
            writeObservation(env, *state, reading, actor, criticOut, training());
        } else {
            std::fill(parts, parts + RewardCount, 0.0f);
            parts[RewardCount - 1] = -task.reward.fall;
            rewards[index] = -task.reward.fall;
            std::fill(actor, actor + O, 0.0f);
            std::fill(criticOut, criticOut + C, 0.0f);
        }
        env.episodeReturn += rewards[index];
        terminated[index] =
            fell || reason == TrainingEndReason3D::Numeric || reason == TrainingEndReason3D::TargetMissed ? 1u : 0u;
        truncated[index] =
            reason == TrainingEndReason3D::TimeLimit || reason == TrainingEndReason3D::Boundary ? 1u : 0u;
        if (reason == TrainingEndReason3D::None) continue;

        std::copy(criticOut, criticOut + C, terminalCritic.data() + index * C);
        finishEpisode(scene, index, reason, reading);
        resetEnvironment(index);
        const RagdollState3D* fresh = physics.ragdollStateView(env.ragdoll);
        const BodyReading start = read(*fresh);
        writeObservation(env, *fresh, start, actor, criticOut, training());
    }
}

TrainingBatch3D::TrainingBatch3D(const TrainingBatchSettings3D& settings) : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    if (settings.environmentCount == 0) throw std::invalid_argument("lote de RL sem mundos");
    impl.settings = settings;
    // O professor conduz pela capsula da cena: um boneco por cena.
    if (settings.teacher) impl.settings.environmentsPerScene = 1;
    impl.task = loadTrainingTask3D(settings.taskPath);
    impl.body = loadRLBody3D(impl.task.bodyPath);
    impl.keyBodies = { impl.body.head, impl.body.leftHand, impl.body.rightHand, impl.body.leftFoot,
        impl.body.rightFoot };
    for (const std::string& id : impl.task.pushes.links)
        for (std::size_t i = 0; i < impl.body.profile.links.size(); ++i)
            if (impl.body.profile.links[i].id == id) impl.pushLinks.push_back(static_cast<std::uint32_t>(i));
    impl.configureEpisodeLength();
    impl.resolvePostures();
    // Os eixos do tronco e dos bracos: tudo menos coxas, canelas e pes.
    for (const RLBodyDof3D& dof : impl.body.dofs) {
        const std::string& link = impl.body.profile.links[dof.linkIndex].id;
        const bool leg = link.find("Thigh") != std::string::npos || link.find("Shin") != std::string::npos
            || link.find("Foot") != std::string::npos || link.find("Toe") != std::string::npos;
        impl.upperDof.push_back(leg ? 0 : 1);
    }
    impl.referenceDofs = impl.resolveReference(impl.task.gait);
    impl.buildReference();
    if (impl.task.target.lookahead > 0 && !(impl.imitating() && impl.reference.library))
        throw std::runtime_error("lote de RL: os proximos alvos (target.lookahead) pedem a biblioteca de clipes");
    if (!settings.posture.empty()) {
        impl.forcedPosture = findRLPosture3D(impl.body, settings.posture);
        if (impl.forcedPosture < 0) throw std::runtime_error("lote de RL: o corpo nao tem a postura " + settings.posture);
    }
    impl.buildLayouts();

    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    const std::uint32_t threads = settings.workerThreads > 0 ? settings.workerThreads : std::max(1u, hardware - 2);
    impl.workers = std::max(1u, threads);
    // A thread que chama step() tambem trabalha no parallelFor.
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);

    const std::size_t N = settings.environmentCount;
    impl.observations.assign(N * impl.observationSize(), 0.0f);
    impl.critic.assign(N * impl.criticSize(), 0.0f);
    impl.terminalCritic.assign(N * impl.criticSize(), 0.0f);
    impl.rewards.assign(N, 0.0f);
    impl.components.assign(N * RewardCount, 0.0f);
    impl.terminated.assign(N, 0);
    impl.truncated.assign(N, 0);
    impl.teacherLabels.assign(settings.teacher ? N * impl.actionSize() : 0, 0.0f);
    impl.createWorld();
    reset(settings.seed);
}

TrainingBatch3D::~TrainingBatch3D() = default;

std::size_t TrainingBatch3D::environmentCount() const { return m_impl->environments.size(); }
std::size_t TrainingBatch3D::observationSize() const { return m_impl->observationSize(); }
std::size_t TrainingBatch3D::criticObservationSize() const { return m_impl->criticSize(); }
std::size_t TrainingBatch3D::actionSize() const { return m_impl->actionSize(); }
std::size_t TrainingBatch3D::rewardComponentCount() const { return RewardCount; }
std::uint32_t TrainingBatch3D::workerThreads() const { return m_impl->workers; }
float TrainingBatch3D::policyStepSeconds() const { return static_cast<float>(m_impl->task.actionRepeat) * FixedStep; }
const RLVectorLayout3D& TrainingBatch3D::observationLayout() const { return m_impl->observationLayout; }
const RLVectorLayout3D& TrainingBatch3D::criticObservationLayout() const { return m_impl->criticLayout; }
const RLVectorLayout3D& TrainingBatch3D::actionLayout() const { return m_impl->actionLayout; }
const RLBody3D& TrainingBatch3D::body() const { return m_impl->body; }
const TrainingTask3D& TrainingBatch3D::task() const { return m_impl->task; }

void TrainingBatch3D::updateTask(const TrainingTask3D& task) {
    Impl& impl = *m_impl;
    // Confere antes de mudar: uma tarefa recusada deixa o lote como estava.
    std::vector<Impl::ReferenceDof> reference = impl.resolveReference(task.gait);
    // A imitacao e o alvo mudam a observacao: nao ligam nem desligam com o
    // treino andando (os clipes podem trocar).
    if (task.imitation.enabled != impl.task.imitation.enabled || task.target.enabled != impl.task.target.enabled)
        throw std::runtime_error("lote de RL: a imitacao e o alvo nao ligam nem desligam com o treino andando");
    if (task.target.lookahead != impl.task.target.lookahead || task.imitation.library != impl.task.imitation.library)
        throw std::runtime_error("lote de RL: os proximos alvos e a biblioteca nao mudam com o treino andando");
    bool clipsChanged = task.imitation.clips.size() != impl.task.imitation.clips.size();
    for (std::size_t i = 0; !clipsChanged && i < task.imitation.clips.size(); ++i)
        clipsChanged = task.imitation.clips[i].path != impl.task.imitation.clips[i].path;
    impl.task.imitation = task.imitation;
    impl.task.target = task.target;
    if (clipsChanged) impl.buildReference();
    impl.task.reward = task.reward;
    impl.task.termination = task.termination;
    impl.task.initialState = task.initialState;
    impl.task.randomization = task.randomization;
    impl.task.pushes = task.pushes;
    impl.task.postures = task.postures;
    impl.task.commands = task.commands;
    impl.task.gait = task.gait;
    impl.referenceDofs = std::move(reference);
    impl.resolvePostures();
    impl.pushLinks.clear();
    for (const std::string& id : impl.task.pushes.links)
        for (std::size_t i = 0; i < impl.body.profile.links.size(); ++i)
            if (impl.body.profile.links[i].id == id) impl.pushLinks.push_back(static_cast<std::uint32_t>(i));
}

void TrainingBatch3D::setPosture(const std::string& id) {
    Impl& impl = *m_impl;
    if (id.empty()) {
        impl.forcedPosture = -1;
        return;
    }
    const int index = findRLPosture3D(impl.body, id);
    if (index < 0) throw std::invalid_argument("lote de RL: o corpo nao tem a postura " + id);
    impl.forcedPosture = index;
}

void TrainingBatch3D::setCommand(std::size_t environment, Vec3 command) {
    if (environment >= m_impl->environments.size()) return;
    Environment& env = m_impl->environments[environment];
    env.commandForced = true;
    env.forcedCommand = command;
    env.command = command;
}

void TrainingBatch3D::clearCommand(std::size_t environment) {
    if (environment >= m_impl->environments.size()) return;
    m_impl->environments[environment].commandForced = false;
}

std::vector<std::string> TrainingBatch3D::taskPostureIds() const {
    std::vector<std::string> ids;
    for (const std::uint32_t index : m_impl->taskPostures) ids.push_back(m_impl->body.postures[index].id);
    return ids;
}

void TrainingBatch3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    std::uint64_t state = seed;
    for (Environment& env : impl.environments) env.rng.seed(splitmix64(state));
    const std::size_t O = impl.observationSize(), C = impl.criticSize();
    for (std::uint32_t i = 0; i < impl.environments.size(); ++i) {
        impl.resetEnvironment(i);
        Environment& env = impl.environments[i];
        // No treino, o relogio de cada episodio comeca sorteado: sem isso, os
        // mundos que aguentam ate o fim terminam todos no mesmo passo e as
        // rodadas viram ondas (o mesmo truque do legged_gym).
        if (impl.training())
            env.steps = std::uniform_int_distribution<std::uint32_t>(0, env.limit - 1)(env.rng);
        const RagdollState3D* view = impl.scenes[env.scene].physics->ragdollStateView(env.ragdoll);
        const BodyReading reading = impl.read(*view);
        impl.writeObservation(env, *view, reading, impl.observations.data() + i * O, impl.critic.data() + i * C,
            impl.training());
    }
    std::fill(impl.rewards.begin(), impl.rewards.end(), 0.0f);
    std::fill(impl.terminated.begin(), impl.terminated.end(), 0);
    std::fill(impl.truncated.begin(), impl.truncated.end(), 0);
    impl.finished.clear();
}

void TrainingBatch3D::step(std::span<const float> actions) {
    Impl& impl = *m_impl;
    if (actions.size() != impl.environments.size() * impl.actionSize())
        throw std::invalid_argument("lote de RL: dimensao das acoes incompativel");
    for (const float value : actions)
        if (!std::isfinite(value)) throw std::invalid_argument("lote de RL: acao nao finita");
    const auto begin = std::chrono::steady_clock::now();
    struct Context {
        Impl* impl;
        std::span<const float> actions;
    } context { &impl, actions };
    impl.scheduler->parallelFor(impl.scenes.size(), 1,
        [](std::size_t first, std::size_t last, void* raw) noexcept {
            auto* ctx = static_cast<Context*>(raw);
            for (std::size_t s = first; s < last; ++s) ctx->impl->stepScene(ctx->impl->scenes[s], ctx->actions);
        },
        &context);
    impl.finished.clear();
    for (const Scene& scene : impl.scenes)
        impl.finished.insert(impl.finished.end(), scene.finished.begin(), scene.finished.end());
    impl.physicsSteps += impl.environments.size() * impl.task.actionRepeat;
    impl.lastStepMilliseconds =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

std::span<const float> TrainingBatch3D::observations() const { return m_impl->observations; }
std::span<const float> TrainingBatch3D::criticObservations() const { return m_impl->critic; }
std::span<const float> TrainingBatch3D::rewards() const { return m_impl->rewards; }
std::span<const float> TrainingBatch3D::rewardComponents() const { return m_impl->components; }
std::span<const float> TrainingBatch3D::teacherActions() const { return m_impl->teacherLabels; }
std::span<const std::uint8_t> TrainingBatch3D::terminated() const { return m_impl->terminated; }
std::span<const std::uint8_t> TrainingBatch3D::truncated() const { return m_impl->truncated; }
std::span<const float> TrainingBatch3D::terminalCriticObservations() const { return m_impl->terminalCritic; }
std::span<const TrainingEpisode3D> TrainingBatch3D::finishedEpisodes() const { return m_impl->finished; }

TrainingEnvironmentView3D TrainingBatch3D::view(std::size_t environment) const {
    const Impl& impl = *m_impl;
    TrainingEnvironmentView3D view;
    if (environment >= impl.environments.size()) return view;
    const Environment& env = impl.environments[environment];
    view.state = impl.scenes[env.scene].physics->ragdollStateView(env.ragdoll);
    view.origin = env.origin;
    view.steps = env.steps;
    view.posture = env.posture;
    view.command = env.command;
    view.startHeading = env.startHeading;
    if (env.pushRemaining > 0.0f) {
        view.pushForce = env.pushForce;
        view.pushLink = static_cast<int>(env.pushLink);
    }
    if (impl.targeting()) {
        view.hasTarget = true;
        view.target = env.target;
        view.targetRadius = impl.task.target.radius;
        view.targetsReached = env.targetsReached;
        view.upcomingCount = impl.imitating() && impl.reference.library ? impl.task.target.lookahead : 0u;
        for (std::uint32_t i = 0; i < view.upcomingCount; ++i) view.upcoming[i] = env.upcoming[i];
        view.reachedCount = env.reachedCount;
        for (std::uint32_t i = 0; i < view.reachedCount; ++i) view.reached[i] = env.reachedTargets[i];
    }
    if (view.state) {
        const RagdollProfile3D& profile = impl.body.profile;
        float mass = 0.0f;
        for (std::size_t i = 0; i < profile.links.size() && i < view.state->links.size(); ++i) {
            view.velocity += view.state->links[i].linearVelocity * profile.links[i].massFraction;
            mass += profile.links[i].massFraction;
        }
        if (mass > 0.0f) view.velocity = view.velocity * (1.0f / mass);
        {
            const std::uint32_t head = impl.body.head;
            const Quaternion frame =
                (view.state->links[head].orientation * profile.links[head].modelOrientation.conjugate()).normalized();
            Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
            forward.z = 0.0f;
            const float length = forward.length();
            view.gaze = length > 1.0e-4f ? forward * (1.0f / length) : Vec3 {};
        }
        if (impl.targeting() && env.facingActive) {
            view.hasFacing = true;
            view.facingCommand = env.facingCommand;
        }
        if (impl.imitating()) {
            const Quaternion frame =
                (view.state->links.front().orientation * profile.links.front().modelOrientation.conjugate()).normalized();
            const Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f });
            const ReferenceSegment3D& segment = impl.reference.segments[std::min<std::size_t>(env.cursor.segment,
                impl.reference.segments.size() - 1)];
            const float heading = std::atan2(forward.y, forward.x) + segment.travelRelative;
            view.hasIntent = true;
            view.intent = { std::cos(heading), std::sin(heading), 0.0f };
        }
    }
    return view;
}

void TrainingBatch3D::push(std::size_t environment, std::uint32_t link, Vec3 forceNewtons, float seconds) {
    Impl& impl = *m_impl;
    if (environment >= impl.environments.size() || link >= impl.body.profile.links.size() || !finite(forceNewtons)
        || !(seconds > 0.0f))
        return;
    Environment& env = impl.environments[environment];
    env.pushLink = link;
    env.pushForce = forceNewtons;
    env.pushRemaining = seconds;
}

std::vector<float> TrainingBatch3D::actionRadiansPerUnit() const {
    std::vector<float> result;
    for (const RLBodyDof3D& dof : m_impl->body.dofs) result.push_back(dof.actionScale);
    return result;
}

float TrainingBatch3D::lastStepMilliseconds() const { return m_impl->lastStepMilliseconds; }
std::uint64_t TrainingBatch3D::physicsStepCount() const { return m_impl->physicsSteps; }

} // namespace MatterEngine
