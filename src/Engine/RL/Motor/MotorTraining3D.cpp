#include "Engine/RL/Motor/MotorTraining3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <stdexcept>

namespace MatterEngine {
namespace {
using Json = nlohmann::json;
constexpr float Pi = 3.14159265358979323846f;
constexpr float Degree = Pi / 180;
constexpr float Gravity = 9.81f;
float wrap(float a) { return std::remainder(a, 2 * Pi); }
float value(Vec3 v, int a) { return a == 0 ? v.x : a == 1 ? v.y : v.z; }
void setValue(Vec3 &v, int a, float x) { (a == 0 ? v.x : a == 1 ? v.y : v.z) = x; }
float sample(std::mt19937 &rng, float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
std::string read(const std::filesystem::path &path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("motor: arquivo ausente: " + path.string());
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
Quaternion modelRotation(const RagdollProfile3D &p, Quaternion world, std::size_t link) {
    return (world * p.links[link].modelOrientation.conjugate()).normalized();
}
float yaw(Quaternion q) {
    const Vec3 f = q.rotate({1, 0, 0});
    return std::atan2(f.y, f.x);
}
Quaternion yawRotation(float radians) { return Quaternion::fromAxisAngle({0, 0, 1}, radians); }
Quaternion nlerp(Quaternion a, Quaternion b, float t) {
    const float s = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0 ? -t : t;
    return Quaternion{a.x * (1 - t) + b.x * s, a.y * (1 - t) + b.y * s, a.z * (1 - t) + b.z * s, a.w * (1 - t) + b.w * s}
        .normalized();
}
// Inclinacao de um vetor "para cima" em dois angulos: para a frente (x) e para a esquerda (y).
std::array<float, 2> lean(Vec3 up) { return {std::atan2(up.x, up.z), std::atan2(up.y, up.z)}; }

enum Metric : std::size_t {
    Seconds, Fell, FallCause, Return, Speed, RequestedSpeed, TargetSpeed, VelocityError, Progress, Tracking,
    Drift, ChestError, HeadError, ChestRotation, TargetRotation, Posture, PelvisTilt, PelvisHeight, Trunk, Spine,
    Twist, Head, Neck, PelvisRate, Imitation, ImitationPose, ImitationVelocity, ImitationEnds, ImitationHeight,
    ActionRate, Slip, Still, BaseWidth, TrotLimit, IncomeAlive, IncomeTask, IncomeImitation, CostIdle, CostPosture,
    CostAction, CostSlip, CostStill, Decisions, HeadingError, InMotion, MetricCount
};
static_assert(MetricCount == MotorEpisodeMetricCount3D);
const std::array<const char *, MotorEpisodeMetricCount3D> MetricNames{
    "segundos",        "caiu",           "causa_queda",     "retorno",          "velocidade",
    "velocidade_pedida", "velocidade_alvo", "erro_velocidade", "progresso",       "rastreio",
    "deriva",          "erro_peito",     "erro_cabeca",     "giro_peito",       "giro_pedido",
    "postura",         "excesso_pelve",  "excesso_altura",  "excesso_tronco",   "excesso_coluna",
    "excesso_torcao",  "excesso_cabeca", "excesso_pescoco", "excesso_giro_pelve", "imitacao",
    "imitacao_pose",   "imitacao_velocidade", "imitacao_extremos", "imitacao_altura", "taxa_acao",
    "escorregar",      "quietude",       "base",            "teto_trote",       "renda_vida",
    "renda_tarefa",    "renda_imitacao", "custo_parado",    "custo_postura",    "custo_acao",
    "custo_escorregar", "custo_quietude", "decisoes",       "erro_rumo",        "comeco_em_movimento"};
// Causas de queda (metrica causa_queda): 1 contato de um link que nao e pe, 2 pelve baixa, 3 tronco tombado, 4 numerico.
enum FallCause : int { NoFall = 0, ContactFall = 1, LowPelvis = 2, TrunkTilt = 3, Numeric = 4 };

struct Zone {
    float free = 0, sigma = 1;
};
struct Recipe {
    std::array<float, 3> shares{.1f, .1f, .8f};
    float turnMinSpeed = .4f, turnMaxSpeed = .8f, turnMinAngle = 30 * Degree, turnMaxAngle = 450 * Degree;
    float pauseMin = 1.5f, pauseMax = 4;
    float trotTarget = 4.5f, trotMinimum = .3f, rampSeconds = 2, trotStart = 1;
    float fallPelvis = .55f, fallPelvisSeconds = .1f, fallTrunk = 65 * Degree, fallTrunkSeconds = .2f;
    Zone pelvisTilt{10 * Degree, 10 * Degree}, pelvisRate{1, 1}, spine{12 * Degree, 10 * Degree};
    Zone twist{30 * Degree, 20 * Degree}, head{12 * Degree, 10 * Degree}, neck{15 * Degree, 10 * Degree};
    float diagonalTwist = 25 * Degree;
    float heightMin = .88f, heightMax = 1.04f, heightSigma = .06f;
    float trunkForward = 15 * Degree, trunkBack = 8 * Degree, trunkSide = 8 * Degree, trunkSigma = 10 * Degree;
    float trunkPerSpeed = 1.5f * Degree, trunkAcceleration = 1;
    float imitationWeight = 1;
    std::string idleClip, turnLeftClip, turnRightClip;
    std::array<std::string, 8> trotClips;
    float turnRadiansPerCycle = Pi / 2, fullBlendSpeed = 1.5f, minCadence = .6f, maxCadence = 1.5f;
    float poseKernel = 2, velocityKernel = .1f, endKernel = 40, heightKernel = .05f;
    float alive = .25f, task = .35f, imitation = .4f, idle = .1f, posture = .15f, action = .1f, slip = .3f,
          still = .3f, fall = 1;
    // 20/10, todos opcionais na receita (ausentes = o comportamento anterior):
    // o giro medido pelo pior entre peito e pelve (torcer a coluna nao gira o corpo); a renda do giro pelo
    // rumo e a do trote por progresso x rumo do peito; a postura multiplicando a renda (ou so como custo);
    // a fracao dos episodios de treino que nascem no movimento do clipe.
    bool turnWholeBody = false, turnHeadingGate = false, trotHeadingGate = false, postureGate = true;
    float turnInMotion = 0, trotInMotion = 0;
    // trot.fullStateStart: o comeco no movimento do trote usa o estado inteiro do clipe na fase sorteada (raiz
    // inclinada, altura do voo, velocidades vertical e angular da raiz). Desligado = o comeco antigo, bit a bit.
    bool trotFullState = false;
    // A rigidez e o amortecimento dos motores, multiplicando os do perfil (1 = o perfil).
    float driveStiffness = 1, driveDamping = 1;
    // O clipe nos motores: alvo = centro + raio*tanh(acao + clipDrive*acao_do_clipe + vies), a rede corrige por
    // cima (1 = o clipe inteiro, 0 = a rede sozinha). A chave presente (mesmo 0) acrescenta a observacao clip.drive.
    bool hasClipDrive = false;
    float clipDrive = 0;
    // A renda da imitacao fora do portao do progresso (imitation.ungated): seguir o clipe em pe paga mesmo antes
    // de acertar a velocidade. Ficar parado nao parece o clipe do trote, entao parado continua sem render.
    bool imitationUngated = false;
};
float finite(const Json &j) {
    const float x = j.get<float>();
    if (!std::isfinite(x))
        throw std::runtime_error("motor: valor nao finito na tarefa");
    return x;
}
float positive(const Json &j) {
    const float x = finite(j);
    if (!(x > 0))
        throw std::runtime_error("motor: valor deve ser positivo na tarefa");
    return x;
}
float nonNegative(const Json &j) {
    const float x = finite(j);
    if (x < 0)
        throw std::runtime_error("motor: valor negativo na tarefa");
    return x;
}
Zone zone(const Json &j) { return {nonNegative(j.at("free")) * Degree, positive(j.at("sigma")) * Degree}; }
Recipe loadRecipe(const std::filesystem::path &path) {
    const Json j = Json::parse(read(path));
    if (j.at("schema") != "matter-motor-2")
        throw std::runtime_error("motor: tarefa de outro contrato (esperado matter-motor-2)");
    Recipe r;
    const auto &shares = j.at("shares");
    r.shares = {nonNegative(shares.at("parado")), nonNegative(shares.at("giro")), nonNegative(shares.at("trote"))};
    if (std::abs(r.shares[0] + r.shares[1] + r.shares[2] - 1) > 1e-4f)
        throw std::runtime_error("motor: as cotas dos exercicios devem somar 1");
    const auto &turn = j.at("turn");
    r.turnMinSpeed = positive(turn.at("speed").at(0));
    r.turnMaxSpeed = positive(turn.at("speed").at(1));
    r.turnMinAngle = positive(turn.at("angleDegrees").at(0)) * Degree;
    r.turnMaxAngle = positive(turn.at("angleDegrees").at(1)) * Degree;
    r.pauseMin = positive(turn.at("pauseSeconds").at(0));
    r.pauseMax = positive(turn.at("pauseSeconds").at(1));
    if (r.turnMaxSpeed < r.turnMinSpeed || r.turnMaxAngle < r.turnMinAngle || r.pauseMax < r.pauseMin)
        throw std::runtime_error("motor: faixas do giro invertidas");
    r.turnWholeBody = turn.value("wholeBody", false);
    r.turnHeadingGate = turn.value("headingGate", false);
    r.turnInMotion = turn.contains("startInMotion") ? nonNegative(turn.at("startInMotion")) : 0.f;
    const auto &trot = j.at("trot");
    r.trotTarget = positive(trot.at("targetSpeed"));
    r.trotMinimum = positive(trot.at("minimumSpeed"));
    r.rampSeconds = positive(trot.at("rampSeconds"));
    r.trotStart = positive(trot.at("curriculum").at("start"));
    if (r.trotMinimum > r.trotTarget || r.trotStart > r.trotTarget)
        throw std::runtime_error("motor: velocidades do trote invalidas");
    r.trotHeadingGate = trot.value("headingGate", false);
    r.trotInMotion = trot.contains("startInMotion") ? nonNegative(trot.at("startInMotion")) : 0.f;
    r.trotFullState = trot.value("fullStateStart", false);
    if (r.turnInMotion > 1 || r.trotInMotion > 1)
        throw std::runtime_error("motor: fracao de comecos no movimento acima de 1");
    const auto &fall = j.at("fall");
    r.fallPelvis = positive(fall.at("pelvisHeight"));
    r.fallPelvisSeconds = positive(fall.at("pelvisSeconds"));
    r.fallTrunk = positive(fall.at("trunkDegrees")) * Degree;
    r.fallTrunkSeconds = positive(fall.at("trunkSeconds"));
    if (r.fallPelvis >= 1 || r.fallTrunk >= Pi)
        throw std::runtime_error("motor: limites de queda invalidos");
    if (j.contains("actuator")) {
        r.driveStiffness = positive(j.at("actuator").at("stiffness"));
        r.driveDamping = positive(j.at("actuator").at("damping"));
        if (r.driveStiffness > 10 || r.driveDamping > 5)
            throw std::runtime_error("motor: rigidez (ate 10x) ou amortecimento (ate 5x) dos motores fora da faixa");
    }
    if (j.contains("clipDrive")) {
        r.hasClipDrive = true;
        r.clipDrive = nonNegative(j.at("clipDrive"));
        if (r.clipDrive > 1)
            throw std::runtime_error("motor: clipDrive (o clipe nos motores) deve ficar em [0, 1]");
    }
    const auto &posture = j.at("posture");
    r.postureGate = posture.value("gate", true);
    r.pelvisTilt = zone(posture.at("pelvisTiltDegrees"));
    r.pelvisRate = {nonNegative(posture.at("pelvisRate").at("free")), positive(posture.at("pelvisRate").at("sigma"))};
    r.heightMin = positive(posture.at("pelvisHeight").at("min"));
    r.heightMax = positive(posture.at("pelvisHeight").at("max"));
    r.heightSigma = positive(posture.at("pelvisHeight").at("sigma"));
    if (r.heightMax < r.heightMin)
        throw std::runtime_error("motor: faixa de altura da pelve invertida");
    const auto &trunk = posture.at("trunkDegrees");
    r.trunkForward = nonNegative(trunk.at("forward")) * Degree;
    r.trunkBack = nonNegative(trunk.at("back")) * Degree;
    r.trunkSide = nonNegative(trunk.at("side")) * Degree;
    r.trunkSigma = positive(trunk.at("sigma")) * Degree;
    r.trunkPerSpeed = nonNegative(trunk.at("perMeterPerSecond")) * Degree;
    r.trunkAcceleration = nonNegative(trunk.at("accelerationScale"));
    r.spine = zone(posture.at("spineDegrees"));
    r.twist = zone(posture.at("twistDegrees"));
    r.diagonalTwist = nonNegative(posture.at("twistDegrees").at("diagonal")) * Degree;
    r.head = zone(posture.at("headDegrees"));
    r.neck = zone(posture.at("neckDegrees"));
    const auto &imitation = j.at("imitation");
    r.imitationWeight = nonNegative(imitation.at("weight"));
    r.imitationUngated = imitation.value("ungated", false);
    if (r.imitationWeight > 1)
        throw std::runtime_error("motor: peso da imitacao deve estar entre 0 e 1");
    r.idleClip = imitation.at("idle");
    r.turnLeftClip = imitation.at("turnLeft");
    r.turnRightClip = imitation.at("turnRight");
    if (imitation.at("trots").size() != 8)
        throw std::runtime_error("motor: oito clipes de trote obrigatorios");
    for (std::size_t i = 0; i < 8; ++i)
        r.trotClips[i] = imitation.at("trots").at(i);
    r.turnRadiansPerCycle = positive(imitation.at("turnDegreesPerCycle")) * Degree;
    r.fullBlendSpeed = positive(imitation.at("fullBlendSpeed"));
    r.minCadence = positive(imitation.at("cadence").at(0));
    r.maxCadence = positive(imitation.at("cadence").at(1));
    if (r.maxCadence < r.minCadence)
        throw std::runtime_error("motor: cadencia invertida");
    const auto &kernels = imitation.at("kernels");
    r.poseKernel = positive(kernels.at("pose"));
    r.velocityKernel = positive(kernels.at("velocity"));
    r.endKernel = positive(kernels.at("ends"));
    r.heightKernel = positive(kernels.at("heightMeters"));
    const auto &w = j.at("weights");
    r.alive = nonNegative(w.at("alive"));
    r.task = nonNegative(w.at("task"));
    r.imitation = nonNegative(w.at("imitation"));
    r.idle = nonNegative(w.at("idle"));
    r.posture = nonNegative(w.at("posture"));
    r.action = nonNegative(w.at("actionRate"));
    r.slip = nonNegative(w.at("slip"));
    r.still = nonNegative(w.at("still"));
    r.fall = nonNegative(w.at("fall"));
    return r;
}

// A postura medida numa pose, no referencial do rumo do peito (graus em radianos).
struct PostureMeasure {
    std::array<float, 2> pelvis{}, trunk{}, spine{}, head{}, neck{};
    float twist = 0;
};

// Um quadro de referencia: as 39 coordenadas, as velocidades (na reproducao natural do clipe),
// pes e maos relativos a pelve no rumo dela, a altura da pelve e a rotacao da raiz.
struct ReferenceFrame {
    std::vector<float> q, qd;
    std::array<Vec3, 4> ends{};
    float height = 0;
    Quaternion root;
};
struct ReferenceBank {
    std::vector<ReferenceFrame> frames;
    float duration = 1, nominalSpeed = 0;
};
} // namespace

struct MotorTraining3D::Impl {
    MotorSettings3D settings;
    Recipe recipe;
    RagdollProfile3D profile;
    std::string hash;
    std::vector<Vec3> idle;  // coordenadas do IDLE quadro 0 (o zero da acao)
    Quaternion idleRoot;
    float idleHeight = 1;
    PostureMeasure idlePosture;
    std::uint32_t chest = 0, head = 0, left = 0, right = 0, leftHand = 0, rightHand = 0;
    struct Axis {
        std::uint32_t link;
        int axis;
        float center, radius, bias;
    };
    std::vector<Axis> axes;
    std::vector<std::size_t> spineTwistAxes;
    std::vector<std::string> actions, observations;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;
    // Bancos: 0 idle, 1 giro esquerda, 2 giro direita, 3.. um por clipe de trote distinto.
    std::vector<ReferenceBank> banks;
    std::array<std::size_t, 8> trotBank{};
    std::array<float, 8> trotLimits{};
    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        RagdollHandle3D handle;
        const RagdollState3D *body = nullptr;
        std::mt19937 rng;
        int drill = 0;
        float time = 0, duration = 20, look = 0, rate = 0, remaining = 0, pause = 0, low = 0, tilted = 0;
        float firstYaw = 0, lastYaw = 0, rotation = 0, targetRotation = 0, targetSpeed = 0;
        float idlePhase = 0, clipPhase = 0;
        float rampStart = 0; // segundos de rampa ja vencidos no nascimento (comeco no movimento)
        bool inMotion = false;
        int activeBank = -1;
        Vec3 anchor, targetVelocity, smoothVelocity;
        std::vector<float> obs, finalObs, lastTargets, referenceAction, lastClipTargets;
        std::vector<RagdollDriveTarget3D> drives;
        std::vector<PhysicsBodyState3D> resetLinks;
        std::array<double, MotorEpisodeMetricCount3D> sum{};
        double trackingSum = 0;
        std::uint32_t trackingCount = 0;
        MotorEpisode3D finished;
        bool ready = false;
        std::uint32_t decisions = 0;
    };
    std::vector<World> worlds;

    template <class F> void parallel(F f) {
        struct Context {
            std::atomic<std::size_t> next{0};
            std::size_t count;
            F *fn;
        } c{{0}, worlds.size(), &f};
        scheduler->parallelFor(
            std::min<std::size_t>(settings.threads, worlds.size()), 1,
            [](std::size_t, std::size_t, void *raw) noexcept {
                auto &ctx = *static_cast<Context *>(raw);
                for (auto i = ctx.next.fetch_add(1); i < ctx.count; i = ctx.next.fetch_add(1))
                    (*ctx.fn)(i);
            },
            &c);
    }
    std::uint32_t findLink(const char *id) const {
        for (std::uint32_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == id)
                return i;
        throw std::runtime_error(std::string("motor: perfil sem ") + id);
    }

    // ---- postura ----
    PostureMeasure measurePosture(std::span<const Vec3> positions, std::span<const Quaternion> orientations) const {
        const auto model = [&](std::size_t link) { return modelRotation(profile, orientations[link], link); };
        const Quaternion pelvisQ = model(0), chestQ = model(chest), headQ = model(head);
        const Quaternion inv = yawRotation(-yaw(chestQ));
        PostureMeasure m;
        m.pelvis = lean(inv.rotate(pelvisQ.rotate({0, 0, 1})));
        m.trunk = lean(inv.rotate(positions[head] - positions[0]));
        m.spine = lean((pelvisQ.conjugate() * chestQ).normalized().rotate({0, 0, 1}));
        m.head = lean(inv.rotate(headQ.rotate({0, 0, 1})));
        m.neck = lean((chestQ.conjugate() * headQ).normalized().rotate({0, 0, 1}));
        m.twist = wrap(yaw(chestQ) - yaw(pelvisQ));
        return m;
    }

    struct PostureScore {
        std::array<float, 8> excess{};
        float factor = 1, penalty = 0;
    };
    // A postura contra o IDLE, com as faixas livres abertas pela demanda assinada (velocidade e aceleracao no
    // rumo do peito). O fator multiplica a renda (sem a velocidade de giro da pelve); a penalidade e o custo.
    PostureScore scorePosture(const PostureMeasure &m, Vec3 demand, Vec3 acceleration, bool diagonal, float h,
                              float pelvisRate) const {
        const auto opening = [&](float v, float a) {
            return recipe.trunkPerSpeed * std::max(0.f, v) +
                   recipe.trunkAcceleration * std::atan(std::max(0.f, a) / Gravity);
        };
        const float sagittal = m.trunk[0] - idlePosture.trunk[0], lateral = m.trunk[1] - idlePosture.trunk[1];
        const float forward = recipe.trunkForward + opening(demand.x, acceleration.x);
        const float back = recipe.trunkBack + opening(-demand.x, -acceleration.x);
        const float leftSide = recipe.trunkSide + opening(demand.y, acceleration.y);
        const float rightSide = recipe.trunkSide + opening(-demand.y, -acceleration.y);
        const float trunkExcess = std::hypot(std::max(0.f, sagittal - forward) + std::max(0.f, -back - sagittal),
                                             std::max(0.f, lateral - leftSide) + std::max(0.f, -rightSide - lateral));
        const auto tilt = [](std::array<float, 2> a, std::array<float, 2> b) { return std::hypot(a[0] - b[0], a[1] - b[1]); };
        PostureScore s;
        s.excess = {std::max(0.f, tilt(m.pelvis, idlePosture.pelvis) - recipe.pelvisTilt.free),
                    std::max(0.f, recipe.heightMin - h) + std::max(0.f, h - recipe.heightMax),
                    trunkExcess,
                    std::max(0.f, tilt(m.spine, idlePosture.spine) - recipe.spine.free),
                    std::max(0.f, std::abs(wrap(m.twist - idlePosture.twist)) - recipe.twist.free -
                                      (diagonal ? recipe.diagonalTwist : 0.f)),
                    std::max(0.f, tilt(m.head, idlePosture.head) - recipe.head.free),
                    std::max(0.f, tilt(m.neck, idlePosture.neck) - recipe.neck.free),
                    std::max(0.f, pelvisRate - recipe.pelvisRate.free)};
        const std::array<float, 8> sigma{recipe.pelvisTilt.sigma, recipe.heightSigma, recipe.trunkSigma, recipe.spine.sigma,
                                         recipe.twist.sigma,      recipe.head.sigma,  recipe.neck.sigma,  recipe.pelvisRate.sigma};
        float exponent = 0;
        for (std::size_t i = 0; i < s.excess.size(); ++i) {
            const float z = s.excess[i] / sigma[i];
            exponent += z * z;
            s.penalty += std::sqrt(1 + z * z) - 1; // pseudo-Huber: linear longe da faixa, sempre com gradiente
        }
        // A velocidade de giro da pelve nao corta a renda (so custa): girar e correr mexem a pelve.
        exponent -= (s.excess[7] / sigma[7]) * (s.excess[7] / sigma[7]);
        s.factor = std::exp(-exponent);
        return s;
    }

    // ---- referencias ----
    ReferenceBank loadBank(const std::string &relative) const {
        const auto clip = loadAnimationClip3D((settings.assetsDirectory / relative).string());
        const auto issues = validateAnimationClipForRagdoll3D(clip, profile);
        if (!issues.empty())
            throw std::runtime_error("motor: referencia " + relative + ": " + issues.front().path + ": " +
                                     issues.front().message);
        const auto *root = findAnimationTrack3D(clip, profile.links[0].id);
        if (!root || clip.durationSeconds <= 0)
            throw std::runtime_error("motor: referencia sem raiz/duracao: " + relative);
        ReferenceBank bank;
        bank.duration = clip.durationSeconds;
        bank.nominalSpeed = clip.nominalSpeedMetersPerSecond;
        const bool loop = clip.loops;
        const float rootZ = sampleAnimationTrackSmooth3D(*root, 0, clip.durationSeconds, loop).translationOffsetMeters.z;
        float grounded = 0;
        std::vector<const AnimationTrack3D *> tracks(profile.links.size());
        for (std::size_t j = 1; j < profile.links.size(); ++j)
            if (!(tracks[j] = findAnimationTrack3D(clip, profile.links[j].id)))
                throw std::runtime_error("motor: referencia sem junta: " + relative);
        const auto coordinatesAt = [&](float t) {
            std::vector<Vec3> c(profile.links.size());
            for (std::size_t j = 1; j < c.size(); ++j)
                c[j] = sampleAnimationTrackSmooth3D(*tracks[j], t, clip.durationSeconds, loop).jointPositionRadians;
            return c;
        };
        for (int frame = 0; frame <= 64; ++frame) {
            const float t = clip.durationSeconds * frame / 64.f;
            ReferenceFrame f;
            const auto c = coordinatesAt(t);
            const float before = loop ? t - .002f : std::max(0.f, t - .002f);
            const float after = loop ? t + .002f : std::min(clip.durationSeconds, t + .002f);
            const auto c0 = coordinatesAt(before), c1 = coordinatesAt(after);
            for (const auto &axis : axes) {
                f.q.push_back(value(c[axis.link], axis.axis));
                f.qd.push_back(value(c1[axis.link] - c0[axis.link], axis.axis) / (after - before));
            }
            const auto s = sampleAnimationTrackSmooth3D(*root, t, clip.durationSeconds, loop);
            f.root = (profile.links[0].modelOrientation * s.rotationDelta).normalized();
            const auto pose = rlBodyPose3D(profile, {}, f.root, c);
            const Quaternion back = yawRotation(-yaw(modelRotation(profile, pose.orientations[0], 0)));
            std::size_t n = 0;
            for (auto link : {left, right, leftHand, rightHand})
                f.ends[n++] = back.rotate(pose.positions[link] - pose.positions[0]);
            if (frame == 0) {
                float bottom = std::numeric_limits<float>::max();
                for (auto foot : {left, right}) {
                    const auto &collider = profile.links[foot].collider;
                    for (const auto &corner : ragdollColliderCorners3D(collider))
                        bottom = std::min(bottom, (pose.positions[foot] + pose.orientations[foot].rotate(
                                                                              collider.localPosition +
                                                                              collider.localOrientation.rotate(corner)))
                                                      .z);
                }
                grounded = .003f - bottom;
            }
            f.height = grounded + s.translationOffsetMeters.z - rootZ;
            bank.frames.push_back(std::move(f));
        }
        return bank;
    }
    void prepareBanks(const Recipe &r, std::vector<ReferenceBank> &out, std::array<std::size_t, 8> &trots) const {
        out.clear();
        out.push_back(loadBank(r.idleClip));
        out.push_back(loadBank(r.turnLeftClip));
        out.push_back(loadBank(r.turnRightClip));
        std::map<std::string, std::size_t> seen;
        for (std::size_t d = 0; d < 8; ++d) {
            const auto found = seen.find(r.trotClips[d]);
            if (found != seen.end()) {
                trots[d] = found->second;
                continue;
            }
            out.push_back(loadBank(r.trotClips[d]));
            if (!(out.back().nominalSpeed > 0))
                throw std::runtime_error("motor: clipe de trote sem velocidade nominal: " + r.trotClips[d]);
            trots[d] = seen[r.trotClips[d]] = out.size() - 1;
        }
    }
    // Interpola um banco na fase [0,1).
    void frameAt(const ReferenceBank &bank, float phase, ReferenceFrame &out) const {
        const float at = phase * (bank.frames.size() - 1);
        const auto i = std::min(static_cast<std::size_t>(at), bank.frames.size() - 2);
        const float f = at - i;
        const auto &a = bank.frames[i], &b = bank.frames[i + 1];
        out.q.resize(a.q.size());
        out.qd.resize(a.qd.size());
        for (std::size_t k = 0; k < a.q.size(); ++k) {
            out.q[k] = a.q[k] * (1 - f) + b.q[k] * f;
            out.qd[k] = a.qd[k] * (1 - f) + b.qd[k] * f;
        }
        for (std::size_t k = 0; k < 4; ++k)
            out.ends[k] = a.ends[k] * (1 - f) + b.ends[k] * f;
        out.height = a.height * (1 - f) + b.height * f;
        out.root = a.root;
    }
    // A raiz interpolada entre os quadros (frameAt guarda a do quadro anterior; so o comeco com estado inteiro usa).
    Quaternion rootAt(const ReferenceBank &bank, float phase) const {
        const float at = phase * (bank.frames.size() - 1);
        const auto i = std::min(static_cast<std::size_t>(at), bank.frames.size() - 2);
        return nlerp(bank.frames[i].root, bank.frames[i + 1].root, at - i);
    }
    // A referencia do mundo (idle misturado ao clipe ativo pelo peso w), na fase dada.
    struct Blend {
        int bank = -1;
        float weight = 0, playback = 1, rate = 0; // rate: ciclos/s do clipe ativo
    };
    Blend blendOf(const World &w) const {
        Blend b;
        if (w.drill == 1 && w.rate != 0) {
            b.bank = w.rate > 0 ? 1 : 2;
            b.weight = 1;
            b.rate = std::abs(w.rate) / recipe.turnRadiansPerCycle;
            b.playback = b.rate * banks[b.bank].duration;
        } else if (w.drill >= 2) {
            const auto &bank = banks[trotBank[w.drill - 2]];
            const float speed = w.targetVelocity.length();
            b.bank = static_cast<int>(trotBank[w.drill - 2]);
            b.weight = std::clamp(speed / recipe.fullBlendSpeed, 0.f, 1.f);
            b.playback = std::clamp(speed / bank.nominalSpeed, recipe.minCadence, recipe.maxCadence);
            b.rate = b.playback / bank.duration;
        }
        return b;
    }
    void referenceAt(const World &w, float lookahead, ReferenceFrame &out, ReferenceFrame &scratch) const {
        const Blend b = blendOf(w);
        const auto &idleBank = banks[0];
        // Com clipe ativo, a parte idle da mistura e o quadro 0 (o zero da acao): a rede ve uma fase so.
        frameAt(idleBank, b.bank < 0 ? std::fmod(w.idlePhase + lookahead / idleBank.duration, 1.f) : 0.f, out);
        if (b.bank < 0 || b.weight <= 0)
            return;
        frameAt(banks[b.bank], std::fmod(w.clipPhase + lookahead * b.rate, 1.f), scratch);
        const float k = b.weight;
        for (std::size_t i = 0; i < out.q.size(); ++i) {
            out.q[i] = out.q[i] * (1 - k) + scratch.q[i] * k;
            out.qd[i] = out.qd[i] * (1 - k) + scratch.qd[i] * b.playback * k;
        }
        for (std::size_t i = 0; i < 4; ++i)
            out.ends[i] = out.ends[i] * (1 - k) + scratch.ends[i] * k;
        out.height = out.height * (1 - k) + scratch.height * k;
    }
    struct ImitationScore {
        float total = 0, pose = 0, velocity = 0, ends = 0, height = 0;
    };
    ImitationScore imitate(const World &w, const ReferenceFrame &ref) const {
        const auto &b = *w.body;
        const bool diagonal = w.drill >= 2 && (w.drill - 2) % 2 == 1;
        float pose = 0, velocity = 0;
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto &axis = axes[i];
            const auto &joint = b.joints[axis.link];
            const float dq = joint.positionRadians[axis.axis] - ref.q[i];
            const float dv = joint.velocityRadiansPerSecond[axis.axis] - ref.qd[i];
            velocity += dv * dv;
            // Nas diagonais a pelve gira para o rumo e o tronco compensa (a regra do modo padrao): a torcao nao conta.
            if (diagonal && std::find(spineTwistAxes.begin(), spineTwistAxes.end(), i) != spineTwistAxes.end())
                continue;
            pose += dq * dq;
        }
        const Quaternion back = yawRotation(-yaw(modelRotation(profile, b.links[0].orientation, 0)));
        float ends = 0;
        std::size_t n = 0;
        for (auto link : {left, right, leftHand, rightHand})
            ends += (back.rotate(b.links[link].position - b.links[0].position) - ref.ends[n++]).lengthSquared();
        const float dh = (b.links[0].position.z - ref.height) / recipe.heightKernel;
        ImitationScore m;
        m.pose = std::exp(-recipe.poseKernel * pose);
        m.velocity = std::exp(-recipe.velocityKernel * velocity);
        m.ends = std::exp(-recipe.endKernel * ends);
        m.height = std::exp(-dh * dh);
        m.total = .65f * m.pose + .10f * m.velocity + .15f * m.ends + .10f * m.height;
        return m;
    }
    void tickReference(World &w) const {
        const Blend b = blendOf(w);
        if (b.bank != w.activeBank) {
            w.activeBank = b.bank;
            w.clipPhase = 0;
        }
        w.idlePhase = std::fmod(w.idlePhase + TickSeconds / banks[0].duration, 1.f);
        if (b.bank >= 0)
            w.clipPhase = std::fmod(w.clipPhase + TickSeconds * b.rate, 1.f);
    }

    // ---- comandos ----
    Vec3 travelDirection(const World &w) const {
        const float angle = w.firstYaw + (w.drill - 2) * Pi / 4;
        return {std::cos(angle), std::sin(angle), 0};
    }
    void updateTranslation(World &w) const {
        w.targetVelocity = w.drill >= 2 ? travelDirection(w) * (w.targetSpeed * std::min(1.0f, (w.time + w.rampStart + DecisionSeconds) /
                                                                                                    recipe.rampSeconds))
                                        : Vec3{};
    }
    Vec3 driftVector(const World &w) const {
        Vec3 error = w.anchor - w.body->links[0].position;
        error.z = 0;
        if (w.drill >= 2) {
            const Vec3 direction = travelDirection(w);
            error = error - direction * dot(error, direction);
        }
        return error;
    }
    void tickCommand(World &w) const {
        if (w.drill != 1) {
            w.rate = 0;
            return;
        }
        if (w.remaining == 0) {
            w.rate = 0;
            if ((w.pause -= TickSeconds) > 0)
                return;
            w.remaining = sample(w.rng, recipe.turnMinAngle, recipe.turnMaxAngle) * (sample(w.rng, 0, 1) < .5f ? -1 : 1);
            w.rate = std::copysign(sample(w.rng, recipe.turnMinSpeed, recipe.turnMaxSpeed), w.remaining);
        }
        const float delta = std::copysign(std::min(std::abs(w.rate * TickSeconds), std::abs(w.remaining)), w.remaining);
        w.look += delta;
        w.targetRotation += delta;
        w.remaining -= delta;
        if (std::abs(w.remaining) < 1e-5f) {
            w.remaining = 0;
            w.rate = 0;
            w.pause = sample(w.rng, recipe.pauseMin, recipe.pauseMax);
        }
    }

    // ---- observacao ----
    void observe(World &w) const {
        const auto &b = *w.body;
        const Quaternion root = modelRotation(profile, b.links[0].orientation, 0);
        const Quaternion inv = yawRotation(-yaw(root));
        std::size_t n = 0;
        const auto put = [&](float x) { w.obs[n++] = x; };
        const auto vec = [&](Vec3 v) {
            put(v.x);
            put(v.y);
            put(v.z);
        };
        vec(root.conjugate().rotate({0, 0, 1}));
        vec(inv.rotate(rlBodyCenterOfMassVelocity3D(profile, b.links)));
        vec(inv.rotate(b.links[0].angularVelocity));
        put(b.links[0].position.z / idleHeight);
        const float error = wrap(w.look - yaw(modelRotation(profile, b.links[chest].orientation, chest)));
        put(std::sin(error));
        put(std::cos(error));
        put(w.rate);
        const Vec3 drift = inv.rotate(driftVector(w));
        put(drift.x);
        put(drift.y);
        for (auto link : {chest, head}) {
            const Quaternion q = modelRotation(profile, b.links[link].orientation, link);
            vec(inv.rotate(q.rotate({1, 0, 0})));
            vec(inv.rotate(q.rotate({0, 0, 1})));
        }
        for (auto foot : {left, right}) {
            vec(inv.rotate(b.links[foot].position - b.links[0].position));
            bool contact = false;
            for (const auto &c : b.contacts)
                if (c.linkIndex == foot && c.normal.z > .5f)
                    contact = true;
            put(contact ? 1.0f : 0.0f);
        }
        for (std::size_t a = 0; a < axes.size(); ++a) {
            const Axis &axis = axes[a];
            const auto &j = b.joints[axis.link];
            put((j.positionRadians[axis.axis] - axis.center) / axis.radius);
            put(j.velocityRadiansPerSecond[axis.axis] * .1f);
            put((w.lastTargets[a] - axis.center) / axis.radius);
        }
        // Ganho 3 em relacao ao v/6 antigo: o V2 mostrou que a rede ignora o comando nessa escala.
        const Vec3 command = inv.rotate(w.targetVelocity) / 2.0f;
        put(command.x);
        put(command.y);
        const float phase = w.activeBank >= 0 ? w.clipPhase : w.idlePhase;
        put(std::sin(2 * Pi * phase));
        put(std::cos(2 * Pi * phase));
        if (recipe.hasClipDrive)
            put(recipe.clipDrive);
        // O rotulo do professor: a pose de referencia uma decisao a frente.
        ReferenceFrame ref, scratch;
        referenceAt(w, DecisionSeconds, ref, scratch);
        for (std::size_t a = 0; a < axes.size(); ++a) {
            const auto &axis = axes[a];
            w.referenceAction[a] =
                std::atanh(std::clamp((ref.q[a] - axis.center) / axis.radius, -.999f, .999f)) - axis.bias;
        }
    }

    // ---- episodios ----
    // rootHeight: a pelve nessa altura, mas nunca um colisor (dos pes, com feetOnly) abaixo do chao.
    void placeBody(World &w, std::span<const Vec3> coordinates, Quaternion rootOrientation, bool feetOnly,
                   Vec3 rootVelocity = {}, Vec3 rootSpin = {}, std::span<const Vec3> jointVelocities = {},
                   float rootHeight = -std::numeric_limits<float>::infinity()) {
        rlBodyLinkStates3D(profile, {}, rootOrientation, rootVelocity, rootSpin, coordinates, jointVelocities, w.resetLinks);
        float bottom = 1e6f;
        for (std::uint32_t link = 0; link < profile.links.size(); ++link) {
            if (feetOnly && link != left && link != right)
                continue;
            const auto &def = profile.links[link];
            const auto &s = w.resetLinks[link];
            for (const Vec3 &c : ragdollColliderCorners3D(def.collider))
                bottom = std::min(bottom,
                                  (s.position + s.orientation.rotate(def.collider.localPosition +
                                                                     def.collider.localOrientation.rotate(c)))
                                      .z);
        }
        const float lift = std::max(.003f - bottom, rootHeight - w.resetLinks[0].position.z);
        for (auto &s : w.resetLinks)
            s.position.z += lift;
        w.scene->resetRagdollState(w.handle, w.resetLinks);
        w.body = w.scene->ragdollStateView(w.handle);
    }
    void resetWorld(World &w, std::size_t index) {
        w.time = w.low = w.tilted = w.rotation = w.targetRotation = 0;
        w.remaining = w.rate = 0;
        w.pause = sample(w.rng, recipe.pauseMin, recipe.pauseMax);
        // Cotas por mundos: a queda nao reduz a participacao do exercicio dificil.
        const float quota = (static_cast<float>(index) + .5f) / worlds.size();
        if (settings.drill >= 0)
            w.drill = settings.drill;
        else if (quota < recipe.shares[2])
            w.drill = 2 + std::min(7, static_cast<int>(quota / recipe.shares[2] * 8));
        else
            w.drill = (quota - recipe.shares[2]) < recipe.shares[1] ? 1 : 0;
        w.duration = settings.episodeSeconds * (settings.evaluation ? 1 : sample(w.rng, .8f, 1.2f));
        w.targetSpeed = 0;
        if (w.drill >= 2) {
            const float limit = settings.trotSpeed > 0 ? settings.trotSpeed : trotLimits[w.drill - 2];
            w.targetSpeed = settings.trotSpeed > 0 || settings.evaluation
                                ? limit
                                : sample(w.rng, std::min(recipe.trotMinimum, limit), limit);
        }
        // Comeco no movimento (so no lote de treino misto; nunca na avaliacao, na tela ou na previa): parte dos
        // trotes nasce na passada da referencia ja na velocidade pedida, e parte dos giros ja girando. Sem isso a
        // rede nunca vive os estados do movimento, e a imitacao (que so paga perto do clipe) nao ensina nada.
        w.rampStart = 0;
        w.inMotion = false;
        const float chance = settings.evaluation || settings.drill >= 0 ? 0.f
                             : w.drill == 1                             ? recipe.turnInMotion
                             : w.drill >= 2                             ? recipe.trotInMotion
                                                                        : 0.f;
        if (chance > 0 && sample(w.rng, 0, 1) < chance)
            w.inMotion = true;
        // Nascimento no IDLE alerta, num instante sorteado do proprio clipe de idle.
        w.idlePhase = sample(w.rng, 0, 1);
        w.clipPhase = 0;
        w.activeBank = -1;
        ReferenceFrame frame;
        frameAt(banks[0], w.idlePhase, frame);
        Vec3 rootVelocity{}, rootSpin{};
        float fullHeight = -std::numeric_limits<float>::infinity(), rootClimb = 0;
        std::vector<Vec3> velocities;
        if (w.inMotion) {
            if (w.drill == 1) {
                w.remaining = sample(w.rng, recipe.turnMinAngle, recipe.turnMaxAngle) * (sample(w.rng, 0, 1) < .5f ? -1 : 1);
                w.rate = std::copysign(sample(w.rng, recipe.turnMinSpeed, recipe.turnMaxSpeed), w.remaining);
                w.pause = 0;
                rootSpin = {0, 0, w.rate};
            } else {
                w.rampStart = recipe.rampSeconds;
                w.targetVelocity = {w.targetSpeed, 0, 0}; // so o modulo conta para a mistura; o rumo vem abaixo
            }
            w.activeBank = blendOf(w).bank;
            w.clipPhase = sample(w.rng, 0, 1);
            ReferenceFrame scratch;
            referenceAt(w, 0, frame, scratch);
            if (w.drill >= 2 && recipe.trotFullState && w.activeBank >= 0) {
                // O estado da mistura idle(quadro 0) -> clipe com o mesmo peso k da pose, por diferenca central.
                const Blend b = blendOf(w);
                const auto &idle0 = banks[0].frames[0];
                const auto state = [&](float dt, Quaternion &root, float &height) {
                    float p = w.clipPhase + dt * b.rate;
                    p -= std::floor(p);
                    frameAt(banks[b.bank], p, scratch);
                    root = nlerp(idle0.root, rootAt(banks[b.bank], p), b.weight);
                    height = idle0.height * (1 - b.weight) + scratch.height * b.weight;
                };
                constexpr float h = .01f;
                Quaternion before, after;
                float low, high;
                state(-h, before, low);
                state(h, after, high);
                state(0, frame.root, fullHeight);
                Quaternion turn = after * before.conjugate(); // giro no referencial da raiz sem rumo
                if (turn.w < 0)
                    turn = {-turn.x, -turn.y, -turn.z, -turn.w};
                const float sine = std::sqrt(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
                if (sine > 1e-7f)
                    rootSpin = Vec3{turn.x, turn.y, turn.z} * (2 * std::atan2(sine, turn.w) / sine / (2 * h));
                rootClimb = (high - low) / (2 * h);
            }
            velocities.assign(profile.links.size(), {});
            for (std::size_t a = 0; a < axes.size(); ++a)
                setValue(velocities[axes[a].link], axes[a].axis, frame.qd[a]);
        }
        std::vector<Vec3> coordinates(idle);
        for (std::size_t a = 0; a < axes.size(); ++a)
            setValue(coordinates[axes[a].link], axes[a].axis, frame.q[a]);
        const float heading = sample(w.rng, -Pi, Pi);
        const Quaternion rootOrientation = (yawRotation(heading) * frame.root).normalized();
        if (w.inMotion && w.drill >= 2) {
            // O rumo da passada sai do peito desta pose, como o firstYaw abaixo.
            const auto pose = rlBodyPose3D(profile, {}, rootOrientation, coordinates);
            const float angle = yaw(modelRotation(profile, pose.orientations[chest], chest)) + (w.drill - 2) * Pi / 4;
            rootVelocity = Vec3{std::cos(angle), std::sin(angle), 0} * w.targetSpeed;
            if (std::isfinite(fullHeight)) {
                rootVelocity.z = rootClimb;
                rootSpin = yawRotation(heading).rotate(rootSpin);
            }
        }
        placeBody(w, coordinates, rootOrientation, true, rootVelocity, rootSpin, velocities, fullHeight);
        w.anchor = w.body->links[0].position;
        w.firstYaw = w.lastYaw = yaw(modelRotation(profile, w.body->links[chest].orientation, chest));
        w.look = w.firstYaw;
        w.smoothVelocity = {};
        if (w.inMotion && w.drill >= 2) {
            const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, w.body->links);
            w.smoothVelocity = {v.x, v.y, 0};
        }
        updateTranslation(w);
        for (std::size_t i = 0; i < axes.size(); ++i)
            w.lastTargets[i] = frame.q[i];
        w.decisions = 0;
        w.sum = {};
        w.trackingSum = 0;
        w.trackingCount = 0;
        w.ready = false;
        observe(w);
    }
    void decide(World &w, std::size_t index, std::span<const float> action, float &reward, std::uint8_t &terminated,
                std::uint8_t &truncated) {
        w.ready = false;
        float change = 0;
        const float clip = recipe.clipDrive;
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const Axis &a = axes[i];
            const float latent = clip > 0 ? action[i] + clip * w.referenceAction[i] : action[i];
            const float target = a.center + a.radius * std::tanh(latent + a.bias);
            float d = (target - w.lastTargets[i]) / .25f;
            if (clip > 0) {
                // So a correcao da rede paga o custo de mudanca, nao o movimento do proprio clipe.
                const float own = a.center + a.radius * std::tanh(clip * w.referenceAction[i] + a.bias);
                d -= (own - w.lastClipTargets[i]) / .25f;
                w.lastClipTargets[i] = own;
            }
            change += d * d;
            w.lastTargets[i] = target;
            w.drives[i].positionRadians = target;
        }
        int cause = NoFall;
        float slip = 0;
        int contacts = 0;
        for (int k = 0; k < 4; ++k) {
            tickCommand(w);
            tickReference(w);
            w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, false);
            w.scene->simulate(TickSeconds);
            w.body = w.scene->ragdollStateView(w.handle);
            const auto &b = *w.body;
            w.low = b.links[0].position.z < idleHeight * recipe.fallPelvis ? w.low + TickSeconds : 0;
            const Vec3 chestUp = modelRotation(profile, b.links[chest].orientation, chest).rotate({0, 0, 1});
            w.tilted = std::acos(std::clamp(chestUp.z, -1.f, 1.f)) > recipe.fallTrunk ? w.tilted + TickSeconds : 0;
            for (const auto &c : b.contacts) {
                if (c.otherMotion == RagdollContactMotion3D::Static && c.linkIndex != left && c.linkIndex != right &&
                    cause == NoFall)
                    cause = ContactFall; // maos, joelhos, tronco ou cabeca no chao: queda (sem andar de quatro)
                if ((c.linkIndex == left || c.linkIndex == right) && c.normal.z > .5f) {
                    const Vec3 v =
                        rlLinkPointVelocity3D(profile.links[c.linkIndex], b.links[c.linkIndex], c.position);
                    slip += (v - c.normal * dot(v, c.normal)).length();
                    ++contacts;
                }
            }
            if (cause == NoFall && w.low >= recipe.fallPelvisSeconds)
                cause = LowPelvis;
            if (cause == NoFall && w.tilted >= recipe.fallTrunkSeconds)
                cause = TrunkTilt;
        }
        const auto &b = *w.body;
        const Vec3 velocity = rlBodyCenterOfMassVelocity3D(profile, b.links);
        const Vec3 flat{velocity.x, velocity.y, 0};
        w.smoothVelocity = w.smoothVelocity + (flat - w.smoothVelocity) * (1 - std::exp(-DecisionSeconds / .25f));
        const float speed = w.smoothVelocity.length();
        const Vec3 position = b.links[0].position;
        const float drift = driftVector(w).length();
        const float requested = w.targetVelocity.length();
        const float velocityError = (w.smoothVelocity - w.targetVelocity).length();
        const Quaternion chestQ = modelRotation(profile, b.links[chest].orientation, chest);
        const Quaternion headQ = modelRotation(profile, b.links[head].orientation, head);
        const float chestError = std::abs(wrap(w.look - yaw(chestQ))), headError = std::abs(wrap(w.look - yaw(headQ)));
        // O giro conta o corpo inteiro: o pior entre o peito e a pelve (torcer a coluna nao gira os pes).
        const float pelvisError = std::abs(wrap(w.look - yaw(modelRotation(profile, b.links[0].orientation, 0))));
        const float headingError = w.drill == 1 && recipe.turnWholeBody ? std::max(chestError, pelvisError) : chestError;
        const float chestYaw = yaw(chestQ);
        w.rotation += wrap(chestYaw - w.lastYaw);
        w.lastYaw = chestYaw;

        // Postura relativa ao IDLE, com as faixas livres abertas pela demanda assinada.
        std::vector<Vec3> positions(b.links.size());
        std::vector<Quaternion> orientations(b.links.size());
        for (std::size_t i = 0; i < b.links.size(); ++i) {
            positions[i] = b.links[i].position;
            orientations[i] = b.links[i].orientation;
        }
        const PostureMeasure m = measurePosture(positions, orientations);
        const Quaternion inv = yawRotation(-chestYaw);
        const Vec3 demand = inv.rotate(w.targetVelocity);
        const Vec3 acceleration = w.drill >= 2 && (w.time + w.rampStart) < recipe.rampSeconds
                                      ? inv.rotate(travelDirection(w)) * (w.targetSpeed / recipe.rampSeconds)
                                      : Vec3{};
        const bool diagonal = w.drill >= 2 && (w.drill - 2) % 2 == 1;
        const float h = position.z / idleHeight;
        const float pelvisRate = std::hypot(b.links[0].angularVelocity.x, b.links[0].angularVelocity.y);
        const PostureScore scored = scorePosture(m, demand, acceleration, diagonal, h, pelvisRate);
        const std::array<float, 8> &excess = scored.excess;
        const float penalty = scored.penalty, posture = scored.factor;

        // Imitacao na fase atual.
        ReferenceFrame ref, scratch;
        referenceAt(w, 0, ref, scratch);
        const ImitationScore imitation = imitate(w, ref);

        // Tarefa.
        const float facing = .5f * (1 + std::cos(chestError));
        const float still = 1 / (1 + (speed / .25f) * (speed / .25f));
        float task = 0, progress = 0;
        // Rumo: metade estreita (~20 graus) para segurar o alvo, metade larga para ter gradiente longe dele.
        const float headingFacing = .5f * (1 + std::cos(headingError));
        const float heading = .5f * std::exp(-(headingError / .35f) * (headingError / .35f)) + .5f * headingFacing;
        if (w.drill == 0)
            task = heading * still / (1 + (drift / .2f) * (drift / .2f));
        else if (w.drill == 1)
            task = heading * still / (1 + (drift / .4f) * (drift / .4f));
        else {
            // Parado sob pedido de movimento rende exatamente zero (regra do usuario).
            progress = requested > 0 ? std::clamp(1 - velocityError / requested, 0.f, 1.f) : 0.f;
            task = facing;
            if ((w.time + w.rampStart) >= recipe.rampSeconds) {
                w.trackingSum += dot(w.smoothVelocity, travelDirection(w)) / std::max(requested, 1e-3f);
                ++w.trackingCount;
            }
        }
        float gate = w.drill >= 2 ? progress : 1.f;
        // A renda so vem com a tarefa: o giro paga pelo rumo do corpo e o trote por progresso de frente para o alvo.
        if (w.drill == 1 && recipe.turnHeadingGate)
            gate = heading;
        else if (w.drill >= 2 && recipe.trotHeadingGate)
            gate = progress * heading;
        // Sem o portao da postura, ela so custa (pseudo-Huber): a forma fica com a imitacao e as quedas.
        const float shape = recipe.postureGate ? posture : 1.f;
        const float actionRate = w.decisions ? change / axes.size() : 0;
        const float meanSlip = contacts ? slip / contacts : 0;
        const float feetSpeed = w.drill == 0 && w.time >= 1.5f
                                    ? .5f * (b.links[left].linearVelocity.length() + b.links[right].linearVelocity.length())
                                    : 0.f;
        const float incomeAlive = shape * gate * recipe.alive;
        const float incomeTask = shape * gate * recipe.task * task;
        const float incomeImitation =
            shape * (recipe.imitationUngated ? 1.f : gate) * recipe.imitation * recipe.imitationWeight * imitation.total;
        const float costIdle = w.drill >= 2 ? recipe.idle * (1 - progress) : 0.f;
        const float costPosture = recipe.posture * penalty;
        const float costAction = recipe.action * actionRate;
        const float costSlip = recipe.slip * meanSlip;
        const float costStill = recipe.still * feetSpeed;
        const float rate = incomeAlive + incomeTask + incomeImitation - costIdle - costPosture - costAction - costSlip - costStill;
        if (cause == NoFall && (!std::isfinite(rate) || !std::isfinite(position.z)))
            cause = Numeric;
        const bool fell = cause != NoFall;
        // A queda soma ao passo: cair nunca escapa dos custos de postura ja pagos naquele passo.
        reward = DecisionSeconds * (std::isfinite(rate) ? rate : 0.f) - (fell ? recipe.fall : 0.f);
        const float base = inv.rotate(b.links[left].position - b.links[right].position).y;
        auto &s = w.sum;
        s[Return] += reward;
        s[Speed] += speed;
        s[RequestedSpeed] += requested;
        s[VelocityError] += velocityError;
        s[Progress] += progress;
        s[ChestError] += chestError;
        s[HeadingError] += headingError;
        s[HeadError] += headError;
        s[Posture] += posture;
        for (std::size_t i = 0; i < excess.size(); ++i)
            s[PelvisTilt + i] += excess[i];
        s[Imitation] += imitation.total;
        s[ImitationPose] += imitation.pose;
        s[ImitationVelocity] += imitation.velocity;
        s[ImitationEnds] += imitation.ends;
        s[ImitationHeight] += imitation.height;
        s[ActionRate] += actionRate;
        s[Slip] += meanSlip;
        s[Still] += feetSpeed;
        s[BaseWidth] += base;
        s[IncomeAlive] += incomeAlive * DecisionSeconds;
        s[IncomeTask] += incomeTask * DecisionSeconds;
        s[IncomeImitation] += incomeImitation * DecisionSeconds;
        s[CostIdle] += costIdle * DecisionSeconds;
        s[CostPosture] += costPosture * DecisionSeconds;
        s[CostAction] += costAction * DecisionSeconds;
        s[CostSlip] += costSlip * DecisionSeconds;
        s[CostStill] += costStill * DecisionSeconds;
        w.time += DecisionSeconds;
        ++w.decisions;
        terminated = fell;
        truncated = !fell && w.time >= w.duration;
        updateTranslation(w);
        observe(w);
        if (terminated || truncated) {
            w.finalObs = w.obs;
            auto &e = w.finished;
            e.world = static_cast<std::uint32_t>(index);
            e.drill = static_cast<std::uint32_t>(w.drill);
            const double n = w.decisions;
            for (std::size_t i = 0; i < MotorEpisodeMetricCount3D; ++i)
                e.values[i] = static_cast<float>(s[i] / n);
            for (std::size_t i = IncomeAlive; i <= CostStill; ++i)
                e.values[i] = static_cast<float>(s[i] / w.time); // taxas por segundo
            e.values[Seconds] = w.time;
            e.values[Fell] = fell ? 1.f : 0.f;
            e.values[FallCause] = static_cast<float>(cause);
            e.values[Return] = static_cast<float>(s[Return]);
            e.values[TargetSpeed] = w.targetSpeed;
            e.values[Tracking] = w.trackingCount ? static_cast<float>(w.trackingSum / w.trackingCount) : 0.f;
            e.values[Drift] = drift;
            e.values[ChestRotation] = w.rotation;
            e.values[TargetRotation] = w.targetRotation;
            e.values[TrotLimit] = w.drill >= 2 ? (settings.trotSpeed > 0 ? settings.trotSpeed : trotLimits[w.drill - 2]) : 0.f;
            e.values[Decisions] = static_cast<float>(w.decisions);
            e.values[InMotion] = w.inMotion ? 1.f : 0.f;
            resetWorld(w, index);
            w.ready = true;
        }
    }
};

MotorTraining3D::MotorTraining3D(const MotorSettings3D &settings) : m_impl(std::make_unique<Impl>()) {
    auto &x = *m_impl;
    x.settings = settings;
    if (!settings.environments || !settings.threads || settings.drill < -1 || settings.drill > 9 ||
        !std::isfinite(settings.episodeSeconds) || settings.episodeSeconds <= 0 || !std::isfinite(settings.trotSpeed))
        throw std::invalid_argument("motor: dimensoes/duracao invalidas");
    const auto path = settings.assetsDirectory / "characters/football_player/FootballPlayerV1.ragdoll.json";
    const auto idlePath = settings.assetsDirectory / "animations/clips/alert_idle.matteranim.json";
    x.profile = loadRagdollProfile3D(path.string());
    const std::string bytes = read(path) + read(idlePath) + "motor-v4";
    std::uint64_t hash = 14695981039346656037ull;
    for (unsigned char c : bytes)
        hash = (hash ^ c) * 1099511628211ull;
    x.hash = std::to_string(hash);
    x.chest = x.findLink("UpperChest");
    x.head = x.findLink("Head");
    x.left = x.findLink("LeftFoot");
    x.right = x.findLink("RightFoot");
    x.leftHand = x.findLink("LeftHand");
    x.rightHand = x.findLink("RightHand");
    // O zero da acao e o IDLE alerta (quadro 0): nascer e comandar zero sao a mesma pose.
    const auto idleClip = loadAnimationClip3D(idlePath.string());
    if (idleClip.targetRigId != x.profile.id || !validateAnimationClipForRagdoll3D(idleClip, x.profile).empty())
        throw std::runtime_error("motor: IDLE alerta incompativel com o corpo");
    x.idle.assign(x.profile.links.size(), {});
    x.idleRoot = x.profile.links[0].modelOrientation;
    for (std::size_t i = 0; i < x.profile.links.size(); ++i) {
        const auto *track = findAnimationTrack3D(idleClip, x.profile.links[i].id);
        if (!track)
            throw std::runtime_error("motor: IDLE sem a junta " + x.profile.links[i].id);
        const auto frame = sampleAnimationTrackSmooth3D(*track, 0.f, idleClip.durationSeconds, idleClip.loops);
        if (i == 0)
            x.idleRoot = (x.profile.links[0].modelOrientation * frame.rotationDelta).normalized();
        else
            x.idle[i] = frame.jointPositionRadians;
    }
    for (std::uint32_t i = 0; i < x.profile.links.size(); ++i) {
        const auto &link = x.profile.links[i];
        for (int a = 0; a < 3; ++a) {
            const auto &d = link.inboundJoint.axes[a];
            if (!d.enabled)
                continue;
            const float center = .5f * (d.minimumRadians + d.maximumRadians),
                        radius = .5f * (d.maximumRadians - d.minimumRadians);
            if (!(radius > 0))
                throw std::runtime_error("motor: eixo sem curso");
            const float q = std::clamp(value(x.idle[i], a), d.minimumRadians + .001f, d.maximumRadians - .001f);
            x.axes.push_back({i, a, center, radius, std::atanh(std::clamp((q - center) / radius, -.999f, .999f))});
            x.actions.push_back(link.id + "." + std::array<const char *, 3>{"twist", "swing1", "swing2"}[a]);
            if (a == 0 && (link.id == "Abdomen" || link.id == "Chest" || link.id == "UpperChest"))
                x.spineTwistAxes.push_back(x.axes.size() - 1);
        }
    }
    if (x.axes.size() != ragdollDegreesOfFreedom3D(x.profile))
        throw std::runtime_error("motor: perfil incompleto");
    {
        const auto pose = rlBodyPose3D(x.profile, {}, x.idleRoot, x.idle);
        x.idlePosture = x.measurePosture(pose.positions, pose.orientations);
        float bottom = 1e6f;
        for (auto foot : {x.left, x.right}) {
            const auto &c = x.profile.links[foot].collider;
            for (const Vec3 &corner : ragdollColliderCorners3D(c))
                bottom = std::min(bottom, (pose.positions[foot] +
                                           pose.orientations[foot].rotate(c.localPosition + c.localOrientation.rotate(corner)))
                                              .z);
        }
        x.idleHeight = .003f - bottom;
    }
    x.recipe = loadRecipe(settings.taskPath);
    x.prepareBanks(x.recipe, x.banks, x.trotBank);
    x.trotLimits.fill(x.recipe.trotStart);
    for (const char *group : {"gravity", "velocity", "angular"})
        for (const char *axis : {"x", "y", "z"})
            x.observations.push_back(std::string(group) + "." + axis);
    for (const char *name : {"height", "target.sin", "target.cos", "target.rate", "anchor.x", "anchor.y"})
        x.observations.push_back(name);
    for (const char *link : {"chest", "head"})
        for (const char *v : {"forward", "up"})
            for (const char *a : {"x", "y", "z"})
                x.observations.push_back(std::string(link) + "." + v + "." + a);
    for (const char *foot : {"left", "right"})
        for (const char *a : {"x", "y", "z", "contact"})
            x.observations.push_back(std::string(foot) + "." + a);
    for (const std::string &name : x.actions)
        for (const char *kind : {"q.", "v.", "a."})
            x.observations.push_back(kind + name);
    for (const char *name : {"command.velocity.x", "command.velocity.y", "reference.phase.sin", "reference.phase.cos"})
        x.observations.push_back(name);
    if (x.recipe.hasClipDrive)
        x.observations.push_back("clip.drive");
    x.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings{std::max(1u, settings.threads - 1)});
    x.engine = std::make_unique<PhysicsEngine3D>(x.scheduler);
    x.worlds.resize(settings.environments);
    PhysicsSceneSettings3D scene;
    scene.runJobsInline = true;
    scene.workerThreadCount = 1;
    scene.maximumBodies = 32;
    scene.maximumBodyPairs = 1024;
    scene.maximumContactConstraints = 1024;
    scene.scratchBufferSizeBytes = 1024 * 1024;
    for (auto &w : x.worlds) {
        w.scene = x.engine->createScene(scene, x.materials);
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = {256, 256, .1f};
        shape.materialId = "concrete";
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = {0, 0, -.1f};
        ground.materialId = "concrete";
        (void)w.scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
        RagdollSpawnDefinition3D spawn;
        spawn.active = true;
        spawn.entityId = 1;
        spawn.pelvisPosition = {0, 0, x.profile.standingRootHeightMeters};
        spawn.solverPositionIterations = 20;
        spawn.solverVelocityIterations = 20;
        w.handle = w.scene->createRagdoll(x.profile, spawn);
        w.body = w.scene->ragdollStateView(w.handle);
        w.obs.resize(x.observations.size());
        w.finalObs.resize(w.obs.size());
        w.lastTargets.resize(x.axes.size());
        w.lastClipTargets.resize(x.axes.size());
        w.referenceAction.resize(x.axes.size());
        for (const auto &a : x.axes) {
            RagdollDriveTarget3D drive;
            drive.linkIndex = a.link;
            drive.axis = static_cast<RagdollAxis3D>(a.axis);
            // O modo ativo do backend multiplica por 5 e 2: rigidez e amortecimento efetivos = os do perfil x receita.
            drive.stiffnessScale = .2f * x.recipe.driveStiffness;
            drive.dampingScale = .5f * x.recipe.driveDamping;
            w.drives.push_back(drive);
        }
    }
    reset(settings.seed);
}
MotorTraining3D::~MotorTraining3D() {
    if (m_impl)
        m_impl->worlds.clear();
}
std::size_t MotorTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t MotorTraining3D::actionSize() const { return m_impl->axes.size(); }
std::size_t MotorTraining3D::observationSize() const { return m_impl->observations.size(); }
const std::vector<std::string> &MotorTraining3D::actionNames() const { return m_impl->actions; }
const std::vector<std::string> &MotorTraining3D::observationNames() const { return m_impl->observations; }
const RagdollProfile3D &MotorTraining3D::profile() const { return m_impl->profile; }
const RagdollState3D &MotorTraining3D::body(std::size_t i) const { return *m_impl->worlds.at(i).body; }
float MotorTraining3D::lookYaw(std::size_t i) const { return m_impl->worlds.at(i).look; }
std::string MotorTraining3D::identity() const { return m_impl->hash; }
float MotorTraining3D::imitationWeight() const { return m_impl->recipe.imitationWeight; }
float MotorTraining3D::clipDrive() const { return m_impl->recipe.clipDrive; }
const std::vector<Vec3> &MotorTraining3D::idleCoordinates() const { return m_impl->idle; }
std::vector<float> MotorTraining3D::actionDerivatives() const {
    std::vector<float> out;
    for (const auto &a : m_impl->axes)
        out.push_back(a.radius * (1 - std::pow(std::tanh(a.bias), 2)));
    return out;
}
std::vector<std::array<float, 2>> MotorTraining3D::actionMapping() const {
    std::vector<std::array<float, 2>> out;
    for (const auto &a : m_impl->axes)
        out.push_back({a.radius, a.bias});
    return out;
}
void MotorTraining3D::poseActions(std::span<const Vec3> coordinates, std::span<float> actions) const {
    const auto &x = *m_impl;
    if (coordinates.size() != x.profile.links.size() || actions.size() != x.axes.size())
        throw std::invalid_argument("motor: dimensoes de pose invalidas");
    for (const auto &v : coordinates)
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
            throw std::invalid_argument("motor: pose nao finita");
    for (std::size_t i = 0; i < x.axes.size(); ++i) {
        const auto &a = x.axes[i];
        actions[i] =
            std::atanh(std::clamp((value(coordinates[a.link], a.axis) - a.center) / a.radius, -.999f, .999f)) - a.bias;
    }
}
void MotorTraining3D::reset(std::uint64_t seed) {
    auto &x = *m_impl;
    x.parallel([&](std::size_t i) {
        x.worlds[i].rng.seed(static_cast<std::uint32_t>(seed + i * 104729));
        x.resetWorld(x.worlds[i], i);
    });
}
void MotorTraining3D::placePose(std::size_t world, std::span<const Vec3> coordinates, Quaternion rootOrientation) {
    auto &x = *m_impl;
    if (coordinates.size() != x.profile.links.size())
        throw std::invalid_argument("motor: pose de outro corpo");
    auto &w = x.worlds.at(world);
    x.placeBody(w, coordinates, rootOrientation.normalized(), false);
    w.anchor = w.body->links[0].position;
    x.observe(w);
}
void MotorTraining3D::observations(std::span<float> out, bool final) const {
    if (out.size() != environmentCount() * observationSize())
        throw std::invalid_argument("motor: buffer de observacao errado");
    for (std::size_t i = 0; i < environmentCount(); ++i) {
        const auto &w = m_impl->worlds[i];
        const auto &src = final ? w.finalObs : w.obs;
        std::copy(src.begin(), src.end(), out.begin() + i * observationSize());
    }
}
void MotorTraining3D::referenceActions(std::span<float> out) const {
    if (out.size() != environmentCount() * actionSize())
        throw std::invalid_argument("motor: buffer de referencia errado");
    for (std::size_t i = 0; i < environmentCount(); ++i) {
        const auto &src = m_impl->worlds[i].referenceAction;
        std::copy(src.begin(), src.end(), out.begin() + i * actionSize());
    }
}
void MotorTraining3D::step(std::span<const float> a, std::span<float> r, std::span<std::uint8_t> t,
                           std::span<std::uint8_t> u) {
    if (a.size() != environmentCount() * actionSize() || r.size() != environmentCount() || t.size() != r.size() ||
        u.size() != r.size())
        throw std::invalid_argument("motor: buffers errados");
    for (float v : a)
        if (!std::isfinite(v))
            throw std::invalid_argument("motor: acao nao finita");
    m_impl->parallel([&](std::size_t i) {
        m_impl->decide(m_impl->worlds[i], i, a.subspan(i * actionSize(), actionSize()), r[i], t[i], u[i]);
    });
}
std::vector<MotorEpisode3D> MotorTraining3D::takeEpisodes() {
    std::vector<MotorEpisode3D> out;
    for (auto &w : m_impl->worlds)
        if (w.ready) {
            out.push_back(w.finished);
            w.ready = false;
        }
    return out;
}
Vec3 MotorTraining3D::commandVelocity(std::size_t i) const { return m_impl->worlds.at(i).targetVelocity; }
const std::array<std::string, 10> &MotorTraining3D::drillIds() {
    static const std::array<std::string, 10> ids{"parado",         "giro",
                                                 "trote_frente",   "trote_frente_esquerda",
                                                 "trote_esquerda", "trote_tras_esquerda",
                                                 "trote_tras",     "trote_tras_direita",
                                                 "trote_direita",  "trote_frente_direita"};
    return ids;
}
const std::array<std::string, 10> &MotorTraining3D::drillNames() {
    static const std::array<std::string, 10> names{"Ficar parado",     "Girar parado",
                                                   "Trote · frente",   "Trote · diagonal frente-esquerda",
                                                   "Trote · esquerda", "Trote · diagonal trás-esquerda",
                                                   "Trote · trás",     "Trote · diagonal trás-direita",
                                                   "Trote · direita",  "Trote · diagonal frente-direita"};
    return names;
}
const std::array<const char *, MotorEpisodeMetricCount3D> &MotorTraining3D::episodeMetricNames() { return MetricNames; }
void MotorTraining3D::setTask(const std::filesystem::path &path) {
    auto recipe = loadRecipe(path);
    if (recipe.hasClipDrive != m_impl->recipe.hasClipDrive)
        throw std::invalid_argument("motor: tirar ou por a chave clipDrive muda as observacoes (para desligar, clipDrive 0)");
    std::vector<ReferenceBank> banks;
    std::array<std::size_t, 8> trots{};
    m_impl->prepareBanks(recipe, banks, trots);
    m_impl->recipe = std::move(recipe);
    m_impl->banks = std::move(banks);
    m_impl->trotBank = trots;
    for (auto &w : m_impl->worlds) {
        w.activeBank = -1;
        w.clipPhase = 0;
        for (auto &drive : w.drives) {
            drive.stiffnessScale = .2f * m_impl->recipe.driveStiffness;
            drive.dampingScale = .5f * m_impl->recipe.driveDamping;
        }
        m_impl->observe(w);
    }
}
void MotorTraining3D::setThreads(std::uint32_t threads) {
    if (!threads || threads > 256)
        throw std::invalid_argument("motor: threads invalidas");
    m_impl->scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings{std::max(1u, threads - 1)});
    m_impl->settings.threads = threads;
}
void MotorTraining3D::setLookYaw(std::size_t i, float radians) {
    if (!std::isfinite(radians))
        throw std::invalid_argument("motor: rumo invalido");
    auto &w = m_impl->worlds.at(i);
    w.look = radians;
    w.rate = 0;
    w.remaining = 0;
    w.pause = 1e6f;
    m_impl->observe(w);
}
void MotorTraining3D::setTrotLimits(std::span<const float> limits) {
    if (limits.size() != 8)
        throw std::invalid_argument("motor: oito tetos de trote");
    for (float v : limits)
        if (!std::isfinite(v) || v < .1f || v > 12)
            throw std::invalid_argument("motor: teto de trote invalido");
    std::copy(limits.begin(), limits.end(), m_impl->trotLimits.begin());
}
std::array<float, 8> MotorTraining3D::trotLimits() const { return m_impl->trotLimits; }
void MotorTraining3D::setShares(std::span<const float> shares) {
    if (shares.size() != 3)
        throw std::invalid_argument("motor: tres cotas (parado, giro, trote)");
    float sum = 0;
    for (float v : shares) {
        if (!std::isfinite(v) || v < 0)
            throw std::invalid_argument("motor: cota invalida");
        sum += v;
    }
    if (std::abs(sum - 1) > 1e-4f)
        throw std::invalid_argument("motor: as cotas devem somar 1");
    std::copy(shares.begin(), shares.end(), m_impl->recipe.shares.begin());
}
std::array<float, 3> MotorTraining3D::shares() const { return m_impl->recipe.shares; }
std::string MotorTraining3D::referencePostureAudit(float trotSpeed) const {
    const auto &x = *m_impl;
    static const std::array<const char *, 8> regions{"pelve", "altura", "tronco", "coluna", "torcao", "cabeca", "pescoco", "giro_pelve"};
    struct Tally {
        double factor = 0, worst = 1, low = 0, n = 0;
        std::array<double, 8> excess{};
    };
    const auto add = [&](Tally &t, const std::vector<float> &q, Quaternion root, float height, Vec3 demand, bool diagonal) {
        std::vector<Vec3> coordinates(x.idle);
        for (std::size_t a = 0; a < x.axes.size(); ++a)
            setValue(coordinates[x.axes[a].link], x.axes[a].axis, q[a]);
        const auto pose = rlBodyPose3D(x.profile, {}, root, coordinates);
        const auto s = x.scorePosture(x.measurePosture(pose.positions, pose.orientations), demand, {}, diagonal,
                                      height / x.idleHeight, 0.f);
        t.factor += s.factor;
        t.worst = std::min<double>(t.worst, s.factor);
        t.low += s.factor < .5f ? 1 : 0;
        for (std::size_t i = 0; i < 8; ++i)
            t.excess[i] += s.excess[i];
        ++t.n;
    };
    const auto report = [&](const Tally &t) {
        Json j = {{"fator_medio", t.factor / t.n}, {"fator_pior", t.worst}, {"quadros_abaixo_de_0_5", t.low / t.n}};
        for (std::size_t i = 0; i < 8; ++i)
            j["excesso_medio"][regions[i]] = t.excess[i] / t.n * (i == 1 || i == 7 ? 1.0 : 180.0 / Pi);
        return j;
    };
    Json out;
    ReferenceFrame idle0;
    x.frameAt(x.banks[0], 0, idle0);
    {
        Tally t;
        for (const auto &f : x.banks[0].frames)
            add(t, f.q, f.root, f.height, {}, false);
        out["parado"]["clipe"] = report(t);
    }
    for (int bank = 1; bank <= 2; ++bank) {
        Tally t;
        for (const auto &f : x.banks[bank].frames)
            add(t, f.q, f.root, f.height, {}, false);
        out["giro"][bank == 1 ? "clipe_esquerda" : "clipe_direita"] = report(t);
    }
    for (int d = 0; d < 8; ++d) {
        const auto &bank = x.banks[x.trotBank[d]];
        const bool diagonal = d % 2 == 1;
        const Vec3 direction{std::cos(d * Pi / 4), std::sin(d * Pi / 4), 0};
        Tally nominal, requested;
        const float k = std::clamp(trotSpeed / x.recipe.fullBlendSpeed, 0.f, 1.f);
        for (const auto &f : bank.frames) {
            add(nominal, f.q, f.root, f.height, direction * bank.nominalSpeed, diagonal);
            std::vector<float> q(f.q.size());
            for (std::size_t i = 0; i < q.size(); ++i)
                q[i] = idle0.q[i] * (1 - k) + f.q[i] * k;
            add(requested, q, idle0.root, idle0.height * (1 - k) + f.height * k, direction * trotSpeed, diagonal);
        }
        auto &j = out[drillIds()[d + 2]];
        j["clipe_na_velocidade_nominal"] = report(nominal);
        j["clipe_na_velocidade_nominal"]["velocidade"] = bank.nominalSpeed;
        j["referencia_pedida"] = report(requested);
        j["referencia_pedida"]["velocidade"] = trotSpeed;
    }
    return out.dump();
}
std::string MotorTraining3D::frameLines(std::size_t i) const {
    std::ostringstream out;
    const auto &b = body(i);
    for (std::size_t j = 0; j < b.links.size(); ++j) {
        const auto &s = b.links[j];
        const auto &p = profile().links[j];
        const Vec3 c = s.position + s.orientation.rotate(p.collider.localPosition);
        const Quaternion q = (s.orientation * p.collider.localOrientation).normalized();
        out << "B " << p.id << ' ' << c.x << ' ' << c.y << ' ' << c.z << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' '
            << q.w << '\n';
    }
    return out.str();
}
} // namespace MatterEngine
