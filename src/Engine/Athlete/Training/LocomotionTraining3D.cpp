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

// Os corpos fixos da avaliacao (indices na grade): o padrao, os 4 cantos e os
// meios das bordas, cada um o ponto da grade mais perto (sem repetir).
std::vector<std::size_t> evaluationBodies(const std::vector<PlayerBody3D>& grid, const LocomotionBodyRange3D& range) {
    const float h0 = range.minimumHeight, h1 = range.maximumHeight, m0 = range.minimumMass, m1 = range.maximumMass;
    const float hm = 0.5f * (h0 + h1), mm = 0.5f * (m0 + m1);
    const PlayerBody3D wanted[] = { {}, { h0, m0 }, { h0, m1 }, { h1, m0 }, { h1, m1 }, { h0, mm }, { h1, mm },
        { hm, m0 }, { hm, m1 } };
    std::vector<std::size_t> out;
    for (const PlayerBody3D& target : wanted) {
        std::size_t best = 0;
        float bestDistance = 1.0e30f;
        for (std::size_t i = 0; i < grid.size(); ++i) {
            const float dh = (grid[i].heightMeters - target.heightMeters) / 0.10f;
            const float dm = (grid[i].massKg - target.massKg) / 10.0f;
            if (dh * dh + dm * dm < bestDistance) {
                bestDistance = dh * dh + dm * dm;
                best = i;
            }
        }
        if (std::find(out.begin(), out.end(), best) == out.end()) out.push_back(best);
    }
    return out;
}

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
    runtime.getUpStiffnessPercent = task.getUpStiffnessPercent;
    runtime.getUpStrengthPercent = task.getUpStrengthPercent;
    // A virada brusca: o episodio liga ou nao (o dropout, no recomeco).
    runtime.dynamicTurnMoveSettings.amount = task.turnMove.amount;
    // A tabela de velocidades do padrao (o piloto e o atleta leem a mesma).
    runtime.defaultTrotSpeed = task.speeds.trot;
    runtime.defaultSprintForwardSpeed = task.speeds.sprintForward;
    runtime.defaultSprintSideSpeed = task.speeds.sprintSide;
    runtime.defaultSprintBackwardSpeed = task.speeds.sprintBackward;
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
            "deriva_espera", "afastamento_curva", "erro_vel_janela", "erro_vel_cruzeiro", "tremor" };
        for (const char* term : LocomotionRewardTermNames3D) c.push_back(std::string("nota_") + term);
        for (const char* name : { "postura_tronco", "postura_pescoco", "pescoco_corpo", "pescoco_sombra",
                 "pescoco_relativo", "viavel_medio", "trancos_teto", "queda_desde_troca", "comeco_guardado",
                 "ajuda_nivel", "celula_queda", "levantou_s", "empurrao_tipo", "empurrao_newtons",
                 "empurrao_segundos", "corpo_altura", "corpo_massa" })
            c.push_back(name);
        for (const char* field : { "decisoes", "cruzeiro", "rastreio", "erro2", "pedida", "feita", "fronteira",
                 "rastreio_fronteira" })
            for (std::size_t k = 0; k < LocomotionCommandCells3D; ++k)
                c.push_back("cel" + std::string(k < 10 ? "0" : "") + std::to_string(k) + "." + field);
        return c;
    }();
    return columns;
}

std::vector<float> locomotionEpisodeRow3D(const LocomotionEpisode3D& e) {
    std::vector<float> row { static_cast<float>(e.environment), static_cast<float>(e.drill),
        static_cast<float>(static_cast<int>(e.reason)), static_cast<float>(e.steps), e.seconds, e.episodeReturn,
        e.velocityError, e.facingError, e.styleError, e.contactPerTouchdown, e.touchdownsPerSecond, e.slip, e.power,
        e.assist, e.posture, e.residual, e.jerk, e.meanRequested, e.meanSpeed, e.distance, e.targetsPerMinute,
        e.holdDrift, e.arcError, e.velocityErrorWindow, e.velocityErrorCruise, e.tremor };
    row.insert(row.end(), e.terms.begin(), e.terms.end());
    for (float v : { e.postureTrunk, e.postureNeck, e.neckBody, e.neckShadow, e.neckRelative, e.meanFeasible, e.jerkCap,
             e.fallSinceChange, e.savedStart, e.assistLevel, e.fallCell, e.stoodSeconds, e.pushKind, e.pushNewtons,
             e.pushSeconds, e.bodyHeight, e.bodyMass })
        row.push_back(v);
    for (const auto* cells : { &e.cellDecisions, &e.cellCruise, &e.cellTracking, &e.cellError2, &e.cellAsked, &e.cellMade,
             &e.cellFrontierCruise, &e.cellFrontierTracking })
        row.insert(row.end(), cells->begin(), cells->end());
    return row;
}

struct LocomotionTraining3D::Impl {
    LocomotionTrainingSettings3D settings;
    LocomotionTask3D task;
    std::shared_ptr<const AthleteBundle3D> bundle;
    std::unique_ptr<LocomotionContract3D> contract;
    // O corpo variavel (task.body): a grade e, por corpo, o pacote, o contrato
    // (o mesmo layout) e o teto fisico de cada canal; vazios sem o bloco (o
    // pacote e o contrato de sempre). Na avaliacao com a grade, os corpos fixos
    // (indices na grade, pelo indice do mundo).
    std::vector<PlayerBody3D> grid;
    std::vector<std::shared_ptr<const AthleteBundle3D>> bodyBundles;
    std::vector<std::unique_ptr<LocomotionContract3D>> bodyContracts;
    std::vector<std::vector<float>> bodyCapMax;
    std::vector<std::size_t> evaluationBodies;
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
    // O teto de cada canal sobre a escala maxima do contrato (a unidade da
    // rede: os trancos na unidade do teto).
    std::vector<float> capFraction;
    std::vector<float> boundLower, boundUpper;
    // Os comecos em estados guardados: o banco por exercicio (o instante e a
    // camera dele) e o sorteio da troca quando o banco esta cheio.
    struct SavedStart {
        AthleteSnapshot3D snapshot;
        float camera = 0.0f;
        // A ultima acao da rede no instante (os trancos do comeco seguem dela).
        std::vector<float> lastAction, lastActionCap;
        // O filtro do candidato: a celula e o rumo de quando ele nasceu.
        int cell = -1;
        // O exercicio dele (o banco certo: o mundo pode ja ter recomecado noutro).
        std::uint32_t drill = 0;
        // O corpo do instante (o mundo recomeca nele).
        std::size_t body = 0;
    };
    std::vector<std::vector<SavedStart>> savedBank;
    std::mt19937_64 bankRng;
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

    struct World {
        std::mt19937_64 rng;
        // O corpo do episodio (indice na grade) e o sorteio so dele.
        std::mt19937_64 bodyRng;
        std::size_t body = 0;
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
        // A quietude: o tremor pesado pela quietude e o peso, no episodio; e o
        // giro do peito do tick anterior (para a aceleracao dele).
        double quietShake = 0.0, quietWeight = 0.0;
        Vec3 lastChestOmega;
        bool lastChestOmegaValid = false;
        double requested = 0.0, speed = 0.0;
        std::array<double, LocomotionRewardTermCount3D> terms {};
        bool hasEpisode = false;
        LocomotionEpisode3D episode;
        // O diagnostico de 18/10 (so medida).
        std::vector<float> lastActionCap;
        double postureTrunk = 0.0, postureNeck = 0.0, neckBody = 0.0, neckShadow = 0.0, neckRelative = 0.0;
        int neckShadowSamples = 0;
        double feasibleSpeed = 0.0, jerkCap = 0.0;
        bool referenceNeckValid = false;
        Vec3 referenceHeadUpInChest { 0.0f, 0.0f, 1.0f };
        float referenceCycleSeconds = -1.0f;
        // O instante do ultimo pouso (s no episodio; o contato por ciclo).
        float lastTouchdown = -1.0f;
        int currentCell = -1;
        std::array<double, LocomotionCommandCells3D> cellDecisions {}, cellCruise {}, cellTracking {}, cellError2 {},
            cellAsked {}, cellMade {}, cellFrontierCruise {}, cellFrontierTracking {};
        // Os ticks da ultima decisao (a queda no tick em que aconteceu).
        int lastDecisionTicks = 0;
        bool savedStart = false;
        float assistLevel = 0.0f;
        // A pancada em curso (v4): a forca (N, mundo) e os ticks que faltam.
        Vec3 pushForce;
        int pushTicks = 0;
        // A duracao do empurrao de agora (ticks; o continuo entra e sai suave).
        int pushTotalTicks = 0;
        // O empurrao unico: quando vem e quando o episodio acaba (s no episodio; < 0: nada marcado).
        float pushAt = -1.0f;
        float settleEnd = -1.0f;
        // O ultimo empurrao sorteado no episodio (a medida: tipo, N, s).
        int pushKind = 0;
        float pushNewtons = 0.0f;
        float pushSeconds = 0.0f;
        // O levantar: ja esteve deitado, ha quanto esta de pe e quando levantou.
        bool wasDown = false;
        float uprightSeconds = 0.0f;
        float stoodSeconds = -1.0f;
        // O instante candidato a comeco guardado (vira do banco se o corpo
        // sobreviver surviveSeconds) e o aceito nesta decisao (o lote junta).
        bool hasPending = false;
        SavedStart pending;
        float pendingAge = 0.0f;
        bool hasAccepted = false;
        SavedStart accepted;
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

    // O pacote, o contrato, o teto e a escala do corpo de um mundo.
    [[nodiscard]] const AthleteBundle3D& bundleOf(const World& world) const {
        return bodyBundles.empty() ? *bundle : *bodyBundles[world.body];
    }
    [[nodiscard]] const LocomotionContract3D& contractOf(const World& world) const {
        return bodyContracts.empty() ? *contract : *bodyContracts[world.body];
    }
    [[nodiscard]] const std::vector<float>& capMaxOf(const World& world) const {
        return bodyCapMax.empty() ? capMax : bodyCapMax[world.body];
    }
    [[nodiscard]] float scaleOf(const World& world) const { return bodyScale3D(bundleOf(world).body()); }

    void computeCaps() {
        const LocomotionAuthority3D full = LocomotionAuthority3D::uniform(1.0f, task.caps);
        const std::vector<float> up = contract->physical(std::vector<float>(actionSize, 1.0f), full);
        const std::vector<float> down = contract->physical(std::vector<float>(actionSize, -1.0f), full);
        capMax.assign(actionSize, 0.0f);
        for (std::size_t i = 0; i < actionSize; ++i) capMax[i] = std::max(std::abs(up[i]), std::abs(down[i]));
        authority.caps = task.caps;
        const LocomotionAuthority3D widest = LocomotionAuthority3D::uniform(1.0f, LocomotionCaps3D::full());
        const std::vector<float> mostUp = contract->physical(std::vector<float>(actionSize, 1.0f), widest);
        const std::vector<float> mostDown = contract->physical(std::vector<float>(actionSize, -1.0f), widest);
        capFraction.assign(actionSize, 0.0f);
        boundLower.assign(actionSize, 0.0f);
        boundUpper.assign(actionSize, 0.0f);
        for (std::size_t i = 0; i < actionSize; ++i) {
            const float most = std::max(std::abs(mostUp[i]), std::abs(mostDown[i]));
            capFraction[i] = most > 0.0f ? capMax[i] / most : 0.0f;
            if (!(capFraction[i] > 0.0f)) continue;
            // (O lado satura onde a acao do contrato chega ao teto dele: v = teto/escala.)
            boundUpper[i] = std::abs(mostUp[i]) > 0.0f ? std::abs(up[i]) / std::abs(mostUp[i]) / capFraction[i] : 0.0f;
            boundLower[i] = std::abs(mostDown[i]) > 0.0f ? -std::abs(down[i]) / std::abs(mostDown[i]) / capFraction[i] : 0.0f;
        }
        // (O teto fisico de cada corpo: os canais em metros vao x s.)
        bodyCapMax.clear();
        for (const std::unique_ptr<LocomotionContract3D>& body : bodyContracts) {
            const std::vector<float> bodyUp = body->physical(std::vector<float>(actionSize, 1.0f), full);
            const std::vector<float> bodyDown = body->physical(std::vector<float>(actionSize, -1.0f), full);
            std::vector<float>& cap = bodyCapMax.emplace_back(actionSize, 0.0f);
            for (std::size_t i = 0; i < actionSize; ++i) cap[i] = std::max(std::abs(bodyUp[i]), std::abs(bodyDown[i]));
        }
        savedBank.resize(task.drills.size());
        for (std::vector<SavedStart>& bank : savedBank)
            if (bank.size() > task.savedStarts.capacity) bank.resize(task.savedStarts.capacity);
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

    [[nodiscard]] static Vec3 centerOfMassVelocity(const RagdollProfile3D& profile, const RagdollState3D& body) {
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
        const CharacterGaitSpeeds3D& gaitSpeeds = bundleOf(world).gaitSpeeds();
        if (magnitude > 1.0e-4f) {
            direction = direction * (1.0f / magnitude);
            stick.moveWorld = direction;
            const auto tableSpeed = [&]() {
                const Vec3 table = athleteStickCommand3D(stick, runtime, gaitSpeeds, 1.0e9f)
                                       .intent.requestedVelocityWorld;
                return std::hypot(table.x, table.y);
            };
            float wanted = std::max(0.0f, input.speed) * std::min(1.0f, magnitude);
            // O curriculo de velocidade (so no treino): o teto da celula do pedido.
            if (task.speedCurriculum.enabled && !settings.evaluation && world.drill < task.speedCurriculum.caps.size()) {
                const std::size_t cell = locomotionCommandCell3D(std::atan2(input.stickLeft, input.stickForward),
                    world.pilot.cameraMode != PilotCameraMode3D::Fixed);
                const float cap = task.speedCurriculum.caps[world.drill][cell];
                if (cap > 0.0f) wanted = std::min(wanted, cap);
            }
            float table = tableSpeed();
            // Acima do trote da tabela, o sprint (o jogador segurando a tecla).
            if (wanted > table + 1.0e-3f) {
                stick.sprint = true;
                table = tableSpeed();
            }
            stick.moveWorld = direction * (table > 1.0e-3f ? std::min(1.0f, wanted / table) : 0.0f);
        }
        return athleteStickCommand3D(stick, runtime, gaitSpeeds, scene.telemetry().terrainSpeedLimit);
    }

    void startEpisode(std::size_t index) {
        World& world = worlds[index];
        LocomotionWorld3D& scene = batch->world(index);
        world.drill = pickDrill(world);
        worldDrills[index] = world.drill;
        // O comeco em movimento (so no treino, com o exercicio pedindo e o banco
        // com instantes; desligado, nem o sorteio acontece: o mundo de antes, bit a bit).
        // O corpo do episodio (so com o bloco: sem ele, nem o sorteio). Um sorteio
        // por episodio no gerador so dele (os outros sorteios do mundo seguem os
        // mesmos); na avaliacao com a grade, o fixo do mundo.
        std::size_t drawn = 0;
        if (!bodyBundles.empty())
            drawn = evaluationBodies.empty()
                ? std::uniform_int_distribution<std::size_t>(0, bodyBundles.size() - 1)(world.bodyRng)
                : evaluationBodies[index % evaluationBodies.size()];
        const SavedStart* saved = nullptr;
        if (!settings.evaluation && task.drills[world.drill].savedStartFraction > 0.0f
            && world.drill < savedBank.size() && !savedBank[world.drill].empty()
            && std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) < task.drills[world.drill].savedStartFraction) {
            const std::vector<SavedStart>& bank = savedBank[world.drill];
            if (bodyBundles.empty()) {
                saved = &bank[std::uniform_int_distribution<std::size_t>(0, bank.size() - 1)(world.rng)];
            } else {
                // (So os instantes do corpo sorteado: o banco enche dos corpos
                // que sobrevivem, e herdar o corpo do instante enviesava a faixa
                // para eles. Sem nenhum do corpo, comeca do zero.)
                std::vector<const SavedStart*> same;
                for (const SavedStart& entry : bank)
                    if (entry.body == drawn) same.push_back(&entry);
                if (!same.empty())
                    saved = same[std::uniform_int_distribution<std::size_t>(0, same.size() - 1)(world.rng)];
            }
        }
        if (!bodyBundles.empty()) {
            world.body = drawn;
            scene.setAssets(bodyBundles[world.body]->assets());
        }
        if (saved != nullptr) {
            scene.restore(saved->snapshot);
        } else {
            scene.reset(AthleteStartMode3D::Physical);
            LocomotionContract3D::clear(scene.athlete());
        }
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
        // A ajuda sorteada por episodio (so no treino; o levantar fica no da tarefa).
        world.assistLevel = task.standingAssistPercent;
        scene.runtimeSettings().historicalAssistRuler = task.historicalAssistRuler;
        world.pushForce = {};
        world.pushTicks = 0;
        world.pushTotalTicks = 0;
        world.pushAt = world.settleEnd = -1.0f;
        world.pushKind = 0;
        world.pushNewtons = world.pushSeconds = 0.0f;
        world.wasDown = false;
        world.uprightSeconds = 0.0f;
        world.stoodSeconds = -1.0f;
        // O levantar comeca com a pancada que derruba.
        if (task.drills[world.drill].getUp) {
            const LocomotionDrill3D& drill = task.drills[world.drill];
            std::uniform_real_distribution<float> unit(0.0f, 1.0f);
            const float angle = unit(world.rng) * 6.28318530718f;
            const float newtons = drill.knockdownMinimumNewtons
                + (drill.knockdownMaximumNewtons - drill.knockdownMinimumNewtons) * unit(world.rng);
            world.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
            world.pushTicks = static_cast<int>(std::lround(0.25f / TickSeconds));
            world.pushTotalTicks = world.pushTicks;
        }
        // (So com o empurrao unico ligado: desligado, nem o sorteio acontece.)
        if (task.drills[world.drill].singlePush)
            world.pushAt = task.pushes.startSeconds
                + task.pushes.windowSeconds * std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng);
        {
            // (A ajuda base da tarefa a cada recomeco: um sorteio de antes nao
            // sobra quando o sorteio e desligado andando.)
            AthleteRuntimeSettings3D& runtime = scene.runtimeSettings();
            runtime.e5StandingAssistPercent = task.standingAssistPercent;
            runtime.e5WalkingAssistPercent = task.walkingAssistPercent;
            runtime.physicalFeetAssistPercent = task.feetAssistPercent;
        }
        if (task.assistSchedule.enabled && !settings.evaluation) {
            const LocomotionAssistSchedule3D& schedule = task.assistSchedule;
            const bool zero = std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) < schedule.zeroFraction;
            const float level = zero ? 0.0f
                : std::uniform_real_distribution<float>(schedule.minimumPercent, schedule.maximumPercent)(world.rng);
            AthleteRuntimeSettings3D& runtime = scene.runtimeSettings();
            runtime.e5StandingAssistPercent = level;
            runtime.e5WalkingAssistPercent = level;
            runtime.physicalFeetAssistPercent = level;
            world.assistLevel = level;
        }
        const RagdollState3D& body = scene.body();
        const Vec3 pelvis = body.links.empty() ? Vec3 {} : body.links.front().position;
        world.start = planar(pelvis);
        // (O corpo nasce olhando para +x; a camera perto disso. Na avaliacao
        // tambem, pelo sorteio do mundo - reproduzivel pela semente: com a
        // camera exata em 0, os mundos parados eram copias bit a bit de um so.
        // No comeco guardado, a camera do instante.)
        const float camera = saved != nullptr ? saved->camera
                                              : std::uniform_real_distribution<float>(-0.25f, 0.25f)(world.rng);
        world.pilot = {};
        CommandPilot3D::start(task.drills[world.drill].pilot, world.pilot, world.start, camera, world.rng);
        const Vec3 startVelocity = saved != nullptr ? planar(centerOfMassVelocity(bundleOf(world).profile(), body)) : Vec3 {};
        if (saved != nullptr) CommandPilot3D::follow(task.drills[world.drill].pilot, world.pilot, startVelocity, world.rng);
        world.command = {};
        world.command.facingYawRadians = camera;
        world.command.intent.requestedFacingYawRadians = camera;
        world.command.lookPitchRadians = -0.2f;
        // (No comeco guardado, a viavel e a velocidade de agora: o corpo ja anda.)
        world.feasible = startVelocity;
        world.comVelocity = startVelocity;
        world.comValid = saved != nullptr;
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
        world.quietShake = world.quietWeight = 0.0;
        world.lastChestOmega = {};
        world.lastChestOmegaValid = false;
        world.requested = world.speed = 0.0;
        world.terms = {};
        world.lastActionCap.assign(actionSize, 0.0f);
        if (saved != nullptr && saved->lastAction.size() == actionSize) {
            world.lastAction = saved->lastAction;
            world.lastActionCap = saved->lastActionCap;
        }
        world.lastDecisionTicks = 0;
        world.postureTrunk = world.postureNeck = world.neckBody = world.neckShadow = world.neckRelative = 0.0;
        world.neckShadowSamples = 0;
        world.feasibleSpeed = world.jerkCap = 0.0;
        world.referenceNeckValid = false;
        world.referenceCycleSeconds = -1.0f;
        world.lastTouchdown = -1.0f;
        world.currentCell = -1;
        world.cellDecisions = world.cellCruise = world.cellTracking = world.cellError2 = world.cellAsked = world.cellMade = {};
        world.cellFrontierCruise = world.cellFrontierTracking = {};
        world.savedStart = saved != nullptr;
        world.hasPending = false;
        world.pendingAge = 0.0f;
        for (std::size_t side = 0; side < 2; ++side) {
            const bool down = touching(body, side);
            world.contactTicks[side] = down ? SlipAfterTicks : 0;
            // (No comeco guardado, o pe no ar ja conta o proximo pouso.)
            if (saved != nullptr && !down) world.airSeconds[side] = TouchdownAirSeconds;
        }
        // O comando da primeira decisao (a rede o ve na primeira entrada).
        world.command = pilotCommand(index, world, TickSeconds * TicksPerDecision);
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
            contractOf(world).observe(body, world.command, view, athlete.policyHistory, world.forecast, obs);
        world.referenceNeckValid = written.referenceNeckValid;
        world.referenceHeadUpInChest = written.referenceHeadUpInChest;
        world.referenceCycleSeconds = written.referenceCycleSeconds;
        std::copy(obs.begin(), obs.end(), criticRow.begin());
        std::size_t c = observationSize;
        for (std::size_t d = 0; d < drillCount; ++d) criticRow[c++] = d == world.drill ? 1.0f : 0.0f;
        const float yaw = body.links.empty() ? 0.0f
                                             : locomotionFrameYaw3D(bundleOf(world).profile(), body.links.front().orientation);
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
        // O corpo do mundo: o perfil, o contrato e a escala dos limiares em
        // metros (1 no corpo de hoje: as contas de antes).
        const RagdollProfile3D& profile = bundleOf(world).profile();
        const LocomotionContract3D& bodyContract = contractOf(world);
        const float s = scaleOf(world);
        const float decisionSeconds = TickSeconds * TicksPerDecision;
        world.hasEpisode = false;
        std::span<float> obs(observations.data() + index * observationSize, observationSize);
        std::span<float> criticRow(criticObservations.data() + index * criticSize, criticSize);
        std::span<float> terms(rewardTerms.data() + index * LocomotionRewardTermCount3D, LocomotionRewardTermCount3D);
        terminated[index] = truncated[index] = 0;

        LocomotionStepMeasure3D measure;
        double jerk = 0.0, jerkCap = 0.0;
        for (std::size_t i = 0; i < actionSize; ++i) {
            const float a = std::isfinite(action[i]) ? std::clamp(action[i], -1.0f, 1.0f) : 0.0f;
            jerk += (a - world.lastAction[i]) * (a - world.lastAction[i]);
            world.lastAction[i] = a;
            // Na unidade do teto (a saida da rede sobre o limite do lado dela: +-1 =
            // o teto daquele lado; os canais assimetricos tem limites diferentes).
            float u = capFraction[i] > 0.0f ? a / capFraction[i] : 0.0f;
            u = u >= 0.0f ? (boundUpper[i] > 0.0f ? std::min(1.0f, u / boundUpper[i]) : 0.0f)
                          : (boundLower[i] < 0.0f ? std::max(-1.0f, u / -boundLower[i]) : 0.0f);
            jerkCap += (u - world.lastActionCap[i]) * (u - world.lastActionCap[i]);
            world.lastActionCap[i] = u;
        }
        const double channels = static_cast<double>(std::max<std::size_t>(1, actionSize));
        measure.jerk = static_cast<float>((task.reward.smoothCapUnits ? jerkCap : jerk) / channels);
        world.jerkCap += jerkCap / channels;
        bodyContract.decide(athlete, action);
        // (O comando desta decisao ja foi preparado no fim da anterior: a rede o
        // viu na entrada, como no jogo, onde before() observa o comando atual.)
        // Os eventos da decisao (v4): a ajuda sorteada de novo no meio do
        // episodio (so no treino) e o comeco de uma pancada.
        {
            std::uniform_real_distribution<float> unit(0.0f, 1.0f);
            const LocomotionAssistSchedule3D& schedule = task.assistSchedule;
            if (schedule.enabled && schedule.changesPerSecond > 0.0f && !settings.evaluation
                && unit(world.rng) < 1.0f - std::exp(-schedule.changesPerSecond * decisionSeconds)) {
                const bool zero = unit(world.rng) < schedule.zeroFraction;
                const float level = zero ? 0.0f
                    : std::uniform_real_distribution<float>(schedule.minimumPercent, schedule.maximumPercent)(world.rng);
                AthleteRuntimeSettings3D& runtime = scene.runtimeSettings();
                runtime.e5StandingAssistPercent = runtime.e5WalkingAssistPercent = runtime.physicalFeetAssistPercent = level;
                world.assistLevel = level;
            }
            const LocomotionDrill3D& drill = task.drills[world.drill];
            const LocomotionPushes3D& pushes = task.pushes;
            // 0: nenhum, 1: o curto, 2: o continuo.
            int kind = 0;
            if (drill.singlePush) {
                if (world.pushAt >= 0.0f && world.seconds >= world.pushAt) {
                    world.pushAt = -1.0f;
                    const float rates = drill.holdRate + drill.pushRate;
                    kind = rates > 0.0f && unit(world.rng) * rates < drill.holdRate ? 2 : 1;
                }
            } else if (world.pushTicks <= 0 && world.seconds >= pushes.startSeconds && (drill.pushRate > 0.0f || drill.holdRate > 0.0f)) {
                // (Um sorteio so: o continuo, o curto ou nenhum.)
                const float hold = 1.0f - std::exp(-drill.holdRate * decisionSeconds);
                const float impulse = 1.0f - std::exp(-drill.pushRate * decisionSeconds);
                const float u = unit(world.rng);
                if (u < hold + impulse) kind = u < hold ? 2 : 1;
            }
            if (kind != 0) {
                const bool continuous = kind == 2;
                const float angle = unit(world.rng) * 6.28318530718f;
                const float low = continuous ? drill.holdMinimumNewtons
                    : drill.pushMinimumNewtons >= 0.0f ? drill.pushMinimumNewtons : pushes.minimumNewtons;
                const float high = continuous ? drill.holdMaximumNewtons
                    : drill.pushMaximumNewtons >= 0.0f ? drill.pushMaximumNewtons : pushes.maximumNewtons;
                const float newtons = low + (high - low) * unit(world.rng);
                const float seconds = continuous
                    ? drill.holdMinimumSeconds + (drill.holdMaximumSeconds - drill.holdMinimumSeconds) * unit(world.rng)
                    : pushes.minimumSeconds + (pushes.maximumSeconds - pushes.minimumSeconds) * unit(world.rng);
                world.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
                world.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
                world.pushTotalTicks = world.pushTicks;
                world.pushKind = kind;
                world.pushNewtons = newtons;
                world.pushSeconds = seconds;
                if (drill.singlePush) world.settleEnd = world.seconds + seconds + drill.settleSeconds;
            }
        }
        const AthleteControlView3D before = athleteControlView3D(athlete);
        const float acceleration = std::max(0.5f, before.navigation.acceleration);
        const float deceleration = std::max(0.5f, before.navigation.deceleration);

        double velocityError2 = 0.0, facingError2 = 0.0, slip = 0.0, power = 0.0, posture = 0.0, assist = 0.0;
        double postureTrunk = 0.0, postureNeck = 0.0, neckBody = 0.0, neckRelative = 0.0, madeSpeed = 0.0;
        double smoothError2 = 0.0;
        const float neckSlack = task.reward.neckSlackDegrees * (Pi / 180.0f);
        double shake = 0.0;
        // (O peso e a massa do corpo do mundo: a ajuda em fracao do peso, a potencia por kg.)
        const float weight = Gravity * std::max(1.0f, profile.totalMassKg);
        const float mass = std::max(1.0f, profile.totalMassKg);
        int slipSamples = 0, ticks = 0;
        bool fell = false, numeric = false;
        const Vec3 asked = planar(world.command.intent.requestedVelocityWorld);
        double targetSpeed = 0.0, upright = 0.0;
        const bool getUpDrill = task.drills[world.drill].getUp;
        for (int tick = 0; tick < TicksPerDecision; ++tick) {
            bodyContract.follow(athlete, authority, TickSeconds);
            if (world.pushTicks > 0) {
                // (A divisao da sonda: pelve, abdomen e peito. O empurrao longo -
                // mais de 0,6 s - entra e sai em 0,2 s.)
                float scale = 1.0f;
                if (static_cast<float>(world.pushTotalTicks) * TickSeconds > 0.6f) {
                    const float ramp = 0.2f / TickSeconds;
                    scale = std::min({ 1.0f, static_cast<float>(world.pushTotalTicks - world.pushTicks + 1) / ramp,
                        static_cast<float>(world.pushTicks) / ramp });
                }
                scene.push(0, world.pushForce * (0.20f * scale));
                scene.push(2, world.pushForce * (0.32f * scale));
                scene.push(3, world.pushForce * (0.48f * scale));
                --world.pushTicks;
            }
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
            targetSpeed += std::hypot(world.feasible.x, world.feasible.y);
            const Vec3 com = planar(centerOfMassVelocity(profile, body));
            if (world.comValid) {
                const Vec3 measured = (com - world.comVelocity) * (1.0f / TickSeconds);
                world.measuredAcceleration += (measured - world.measuredAcceleration)
                    * (1.0f - std::exp(-TickSeconds / 0.25f));
            }
            world.comVelocity = com;
            world.comValid = true;
            madeSpeed += std::hypot(com.x, com.y);
            const Vec3 error = com - world.feasible;
            velocityError2 += error.x * error.x + error.y * error.y;
            {
                const float blend = 1.0f - std::exp(-TickSeconds / 0.25f);
                world.comSmooth += (com - world.comSmooth) * blend;
                world.feasibleSmooth += (world.feasible - world.feasibleSmooth) * blend;
                const Vec3 smooth = world.comSmooth - world.feasibleSmooth;
                world.windowError2 += smooth.x * smooth.x + smooth.y * smooth.y;
                smoothError2 += smooth.x * smooth.x + smooth.y * smooth.y;
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
                // O tremor (a quietude): a pelve no plano, o giro horizontal do peito
                // (o balanco e o cabeceio; o giro em torno do eixo de cima fica de fora:
                // virar no lugar nao e tremer) e a aceleracao dele neste tick.
                {
                    const Vec3 omega = planar(body.links[static_cast<std::size_t>(chest)].angularVelocity);
                    const Vec3 pelvisVelocity = planar(body.links.front().linearVelocity);
                    float alpha = 0.0f;
                    if (world.lastChestOmegaValid) {
                        const Vec3 change = omega - world.lastChestOmega;
                        alpha = std::hypot(change.x, change.y) / TickSeconds;
                    }
                    world.lastChestOmega = omega;
                    world.lastChestOmegaValid = true;
                    shake += 0.5 * std::clamp(std::hypot(pelvisVelocity.x, pelvisVelocity.y) / 0.10f, 0.0f, 3.0f)
                        + 0.3 * std::clamp(std::hypot(omega.x, omega.y) / 0.5f, 0.0f, 3.0f)
                        + 0.2 * std::clamp(alpha / 20.0f, 0.0f, 3.0f);
                }
                LocomotionPostureInput3D input;
                input.chestUp = frame.in(r.rotate({ 0.0f, 0.0f, 1.0f }));
                input.measuredAcceleration = frame.in(world.measuredAcceleration);
                Vec3 commanded = (asked - com) * (1.0f / 0.25f);
                const float size = std::hypot(commanded.x, commanded.y);
                commanded = size < 0.3f ? Vec3 {} : commanded * (std::min(size, 8.0f) * 0.5f / size);
                input.commandAcceleration = frame.in(commanded);
                input.speed = std::hypot(com.x, com.y);
                input.neckSlack = neckSlack;
                input.neckRelative = task.reward.neckRelative;
                if (head >= 0 && static_cast<std::size_t>(head) < body.links.size()) {
                    const Quaternion h = (body.links[static_cast<std::size_t>(head)].orientation
                        * profile.links[static_cast<std::size_t>(head)].modelOrientation.conjugate()).normalized();
                    input.neckBend = std::acos(std::clamp(dot(r.rotate({ 0.0f, 0.0f, 1.0f }), h.rotate({ 0.0f, 0.0f, 1.0f })), -1.0f, 1.0f));
                    input.headUpInChest = r.conjugate().rotate(h.rotate({ 0.0f, 0.0f, 1.0f }));
                    input.referenceValid = world.referenceNeckValid;
                    input.referenceHeadUpInChest = world.referenceHeadUpInChest;
                    if (world.referenceNeckValid)
                        neckRelative += std::acos(std::clamp(dot(input.headUpInChest, world.referenceHeadUpInChest), -1.0f, 1.0f));
                }
                const LocomotionPostureParts3D parts = locomotionPostureParts3D(input);
                posture += parts.trunk + parts.neck;
                postureTrunk += parts.trunk;
                postureNeck += parts.neck;
                neckBody += input.neckBend;
            }
            for (std::size_t side = 0; side < 2; ++side) {
                if (feet[side] < 0 || static_cast<std::size_t>(feet[side]) >= body.links.size()) continue;
                const Vec3 foot = body.links[static_cast<std::size_t>(feet[side])].position;
                if (touching(body, side)) {
                    if (world.contactTicks[side] == 0 && world.airSeconds[side] >= TouchdownAirSeconds) {
                        ++world.touchdowns;
                        const float now = world.seconds + static_cast<float>(tick + 1) * TickSeconds;
                        if (world.goalValid[side]) {
                            const float dx = foot.x - world.goal[side].x, dy = foot.y - world.goal[side].y;
                            // (A distancia do pouso x s: o corpo em escala.)
                            const float sigma = task.reward.contactSigma * s;
                            float score = std::exp(-(dx * dx + dy * dy) / (sigma * sigma));
                            if (task.reward.contactPerCycle && world.referenceCycleSeconds > 0.0f && world.lastTouchdown >= 0.0f)
                                score *= std::min(1.0f, (now - world.lastTouchdown) / (0.5f * world.referenceCycleSeconds));
                            measure.contactScore += score;
                            ++world.contactGoals;
                            world.goalValid[side] = false;
                        }
                        world.lastTouchdown = now;
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
            if (task.reward.assistIncludesTorque) {
                const AthleteExternalAssist3D& help = athlete.externalAssist;
                assist += (help.sliderNewtons() + std::abs(help.netTorque) + std::abs(help.postureCoupleTorque)) / weight;
            } else {
                assist += athlete.externalAssist.sliderNewtons() / weight;
            }
            // O quanto esta de pe (o levantar): a pelve entre deitada (0,35 m) e
            // em pe, vezes o peito para cima.
            {
                const float stand = std::max(0.5f, profile.standingRootHeightMeters);
                const float lying = 0.35f * s;
                const float height = std::clamp((body.links.front().position.z - athlete.groundHeight - lying)
                    / (stand - lying), 0.0f, 1.0f);
                float chestUp = 1.0f;
                if (chest >= 0 && static_cast<std::size_t>(chest) < body.links.size()) {
                    const Quaternion r = (body.links[static_cast<std::size_t>(chest)].orientation
                        * profile.links[static_cast<std::size_t>(chest)].modelOrientation.conjugate()).normalized();
                    chestUp = std::clamp(r.rotate({ 0.0f, 0.0f, 1.0f }).z, 0.0f, 1.0f);
                }
                const float u = height * chestUp;
                upright += u;
                if (getUpDrill) {
                    if (u < 0.4f) world.wasDown = true;
                    world.uprightSeconds = u > 0.85f ? world.uprightSeconds + TickSeconds : 0.0f;
                    if (world.wasDown && world.stoodSeconds < 0.0f && world.uprightSeconds >= 0.5f)
                        world.stoodSeconds = world.seconds + static_cast<float>(tick + 1) * TickSeconds;
                }
            }
            // (No levantar, cair nao encerra.)
            if (!getUpDrill && body.links.front().position.z - athlete.groundHeight < task.fallPelvisHeight * s) {
                fell = true;
                break;
            }
            // (v4) Qualquer link que nao seja pe no chao: queda, no tick.
            if (task.reward.forbiddenContact && !getUpDrill) {
                bool forbidden = false;
                for (const RagdollInteraction3D& contact : body.interactions) {
                    const int link = static_cast<int>(contact.linkIndex);
                    if (link == feet[0] || link == feet[1] || link == toes[0] || link == toes[1]) continue;
                    if (contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z > 0.5f) forbidden = true;
                }
                if (forbidden) {
                    fell = true;
                    break;
                }
            }
        }
        if (!numeric && !fell && !getUpDrill && scene.telemetry().fallen) fell = true;
        const float tickCount = static_cast<float>(std::max(1, ticks));
        const float smoothedError = static_cast<float>(std::sqrt(smoothError2 / tickCount));
        measure.velocityError = task.reward.velocitySmoothed ? smoothedError
                                                             : static_cast<float>(std::sqrt(velocityError2 / tickCount));
        measure.facingError = static_cast<float>(std::sqrt(facingError2 / tickCount));
        measure.targetSpeed = static_cast<float>(targetSpeed / tickCount);
        measure.getUpDrill = getUpDrill;
        measure.upright = getUpDrill ? static_cast<float>(upright / tickCount) : 1.0f;
        measure.slip = slipSamples > 0 ? static_cast<float>(slip / slipSamples) : 0.0f;
        measure.power = static_cast<float>(power / tickCount);
        measure.posture = static_cast<float>(posture / tickCount);
        measure.assist = static_cast<float>(assist / tickCount);
        measure.fell = fell;
        world.postureTrunk += postureTrunk / tickCount;
        world.postureNeck += postureNeck / tickCount;
        world.neckBody += neckBody / tickCount;
        if (world.referenceNeckValid) {
            world.neckShadow += std::acos(std::clamp(world.referenceHeadUpInChest.z, -1.0f, 1.0f));
            world.neckRelative += neckRelative / tickCount;
            ++world.neckShadowSamples;
        }
        world.feasibleSpeed += std::hypot(world.feasible.x, world.feasible.y);
        world.lastDecisionTicks = ticks;
        // A celula de comando desta decisao (o rumo do pedido em relacao a camera e
        // o modo dela) e o rastreio nela (o nucleo de velocidade sem peso, no cruzeiro).
        {
            const float askedSpeed = std::hypot(asked.x, asked.y);
            if (askedSpeed > 0.3f) {
                const std::size_t cell = locomotionCommandCell3D(
                    std::atan2(asked.y, asked.x) - world.command.facingYawRadians,
                    world.pilot.cameraMode != PilotCameraMode3D::Fixed);
                world.currentCell = static_cast<int>(cell);
                world.cellDecisions[cell] += 1.0;
                world.cellAsked[cell] += askedSpeed;
                world.cellMade[cell] += madeSpeed / tickCount;
                const Vec3 gap = asked - world.feasible;
                if (std::hypot(gap.x, gap.y) < 0.05f) {
                    const float e = measure.velocityError;
                    const float kernelValue = std::exp(-(e * e) / (task.reward.velocitySigma * task.reward.velocitySigma));
                    world.cellCruise[cell] += 1.0;
                    world.cellTracking[cell] += kernelValue;
                    world.cellError2[cell] += static_cast<double>(e) * e;
                    // A fronteira (o curriculo): o pedido no teto da celula.
                    if (task.speedCurriculum.enabled && world.drill < task.speedCurriculum.caps.size()) {
                        const float cap = task.speedCurriculum.caps[world.drill][cell];
                        if (cap > 0.0f && askedSpeed >= cap - task.speedCurriculum.frontier) {
                            // (O rastreio da fronteira pela velocidade suavizada em ~uma
                            // passada: a oscilacao natural da passada nao e erro.)
                            const float sigma = task.reward.velocitySigma;
                            world.cellFrontierCruise[cell] += 1.0;
                            world.cellFrontierTracking[cell] += std::exp(-(smoothedError * smoothedError) / (sigma * sigma));
                        }
                    }
                }
            } else {
                world.currentCell = -1;
            }
        }
        {
            // A quietude pedida: o pedido ~0 (0,1 a 0,4 m/s a liga e desliga) e sem
            // frear (o pedido perto do viavel: frear nao e tremer).
            const auto smoothstep = [](float a, float b, float x) {
                const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
                return t * t * (3.0f - 2.0f * t);
            };
            const Vec3 gap = asked - world.feasible;
            const float quiet = (1.0f - smoothstep(0.10f, 0.40f, std::hypot(asked.x, asked.y)))
                * (1.0f - smoothstep(0.20f, 0.60f, std::hypot(gap.x, gap.y)));
            const float tremor = static_cast<float>(shake / tickCount);
            measure.still = quiet * tremor;
            world.quietShake += static_cast<double>(quiet) * tremor;
            world.quietWeight += quiet;
        }

        // O comando da proxima decisao, antes da entrada que a rede vai usar.
        world.command = pilotCommand(index, world, decisionSeconds);
        // A entrada de agora (e o que a nota le dela: o estilo e as metas de
        // pouso da referencia).
        const std::size_t nonFinite = observe(index, obs, criticRow);
        if (nonFinite > 0) numeric = true;
        if (obs[referenceValid] > 0.5f) {
            double sum = 0.0, worst = 0.0;
            for (std::size_t part = 0; part < styleError.size(); ++part) {
                double one = 0.0;
                for (std::size_t axis = 0; axis < 3; ++axis)
                    one += obs[styleError[part] + axis] * obs[styleError[part] + axis];
                sum += one;
                worst = std::max(worst, one);
            }
            const double mean = sum / static_cast<double>(styleError.size());
            measure.styleValid = true;
            measure.styleError = static_cast<float>(std::sqrt(task.reward.styleWorstPart ? 0.5 * (mean + worst) : mean));
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
            const std::vector<float>& cap = capMaxOf(world);
            double residual = 0.0;
            std::size_t counted = 0;
            for (std::size_t i = 0; i < actionSize && i < athlete.policyApplied.size(); ++i) {
                if (!(cap[i] > 0.0f)) continue;
                const double r = athlete.policyApplied[i] / cap[i];
                residual += r * r;
                ++counted;
            }
            measure.residual = counted > 0 ? static_cast<float>(residual / static_cast<double>(counted)) : 0.0f;
        }
        // (O erro de estilo e em metros: o sigma dele x s no corpo variavel.)
        LocomotionRewardWeights3D scaledReward;
        const LocomotionRewardWeights3D* reward = &task.reward;
        if (s != 1.0f) {
            scaledReward = task.reward;
            scaledReward.styleSigma *= s;
            reward = &scaledReward;
        }
        const std::array<float, LocomotionRewardTermCount3D> values =
            locomotionRewardTerms3D(measure, *reward, decisionSeconds);
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

        // Os comecos guardados: o candidato vira do banco depois de sobreviver
        // surviveSeconds; um candidato novo so no cruzeiro, rapido e longe do limite.
        if (!settings.evaluation && task.drills[world.drill].savedStartFraction > 0.0f && !fell && !numeric) {
            const RagdollState3D& here = scene.body();
            const Vec3 spot = here.links.empty() ? Vec3 {} : planar(here.links.front().position);
            const Vec3 lag = asked - world.feasible;
            if (world.hasPending) {
                // (O candidato so vale se o corpo seguiu no mesmo pedido: rapido, perto
                // do viavel, na mesma celula e longe do limite durante a espera.)
                const bool still = std::hypot(asked.x, asked.y) >= task.savedStarts.minimumSpeed
                    && std::hypot(lag.x, lag.y) < 0.25f && world.currentCell == world.pending.cell
                    && std::hypot(spot.x, spot.y) < 0.5f * task.boundaryMeters;
                world.pendingAge += decisionSeconds;
                if (!still) {
                    world.hasPending = false;
                } else if (world.pendingAge >= task.savedStarts.surviveSeconds) {
                    world.accepted = std::move(world.pending);
                    world.hasAccepted = true;
                    world.hasPending = false;
                }
            } else {
                const RagdollState3D& now = scene.body();
                const Vec3 gap = asked - world.feasible;
                const Vec3 at = now.links.empty() ? Vec3 {} : planar(now.links.front().position);
                if (std::hypot(asked.x, asked.y) >= task.savedStarts.minimumSpeed && std::hypot(gap.x, gap.y) < 0.05f
                    && std::hypot(at.x, at.y) < 0.5f * task.boundaryMeters
                    && std::uniform_real_distribution<float>(0.0f, 1.0f)(world.rng) < 0.05f) {
                    world.pending.snapshot = scene.snapshot();
                    world.pending.camera = world.command.facingYawRadians;
                    world.pending.lastAction = world.lastAction;
                    world.pending.lastActionCap = world.lastActionCap;
                    world.pending.cell = world.currentCell;
                    world.pending.drill = world.drill;
                    world.pending.body = world.body;
                    world.hasPending = true;
                    world.pendingAge = 0.0f;
                }
            }
        }

        LocomotionEnd3D end = LocomotionEnd3D::None;
        const RagdollState3D& body = scene.body();
        const Vec3 pelvis = body.links.empty() ? Vec3 {} : body.links.front().position;
        if (numeric) end = LocomotionEnd3D::Numeric;
        else if (fell) end = LocomotionEnd3D::Fall;
        else if (world.seconds >= episodeSeconds() - 1.0e-4f
            || (world.settleEnd >= 0.0f && world.seconds >= world.settleEnd - 1.0e-4f))
            end = LocomotionEnd3D::TimeLimit;
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
        e.tremor = world.quietWeight > 1.0 ? static_cast<float>(world.quietShake / world.quietWeight) : -1.0f;
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
        e.postureTrunk = static_cast<float>(world.postureTrunk / steps);
        e.postureNeck = static_cast<float>(world.postureNeck / steps);
        e.neckBody = static_cast<float>(world.neckBody / steps);
        e.neckShadow = world.neckShadowSamples > 0 ? static_cast<float>(world.neckShadow / world.neckShadowSamples) : -1.0f;
        e.neckRelative = world.neckShadowSamples > 0 ? static_cast<float>(world.neckRelative / world.neckShadowSamples) : -1.0f;
        e.meanFeasible = static_cast<float>(world.feasibleSpeed / steps);
        e.jerkCap = static_cast<float>(world.jerkCap / steps);
        e.fallSinceChange = reason == LocomotionEnd3D::Fall
            ? world.pilot.clock - world.pilot.lastChange + static_cast<float>(world.lastDecisionTicks) * TickSeconds
            : -1.0f;
        e.savedStart = world.savedStart ? 1.0f : 0.0f;
        e.assistLevel = world.assistLevel;
        e.stoodSeconds = world.stoodSeconds;
        e.pushKind = static_cast<float>(world.pushKind);
        e.pushNewtons = world.pushNewtons;
        e.pushSeconds = world.pushSeconds;
        e.bodyHeight = bundleOf(world).body().heightMeters;
        e.bodyMass = bundleOf(world).body().massKg;
        e.fallCell = reason == LocomotionEnd3D::Fall && world.currentCell >= 0 ? static_cast<float>(world.currentCell) : -1.0f;
        for (std::size_t k = 0; k < LocomotionCommandCells3D; ++k) {
            e.cellDecisions[k] = static_cast<float>(world.cellDecisions[k]);
            e.cellCruise[k] = static_cast<float>(world.cellCruise[k]);
            e.cellTracking[k] = static_cast<float>(world.cellTracking[k]);
            e.cellError2[k] = static_cast<float>(world.cellError2[k]);
            e.cellAsked[k] = static_cast<float>(world.cellAsked[k]);
            e.cellMade[k] = static_cast<float>(world.cellMade[k]);
            e.cellFrontierCruise[k] = static_cast<float>(world.cellFrontierCruise[k]);
            e.cellFrontierTracking[k] = static_cast<float>(world.cellFrontierTracking[k]);
        }
        // (Um candidato a comeco guardado nao sobrevive ao fim do episodio.)
        world.hasPending = false;
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
    // (Com o corpo variavel, o contrato ganha os canais do jogador; o do corpo de
    // hoje da o layout, a impressao e as escalas.)
    const PlayerBody3D base;
    impl.contract = std::make_unique<LocomotionContract3D>(profile, impl.task.body.enabled ? &base : nullptr);
    impl.grid = locomotionBodyGrid3D(impl.task.body);
    if (impl.task.body.enabled) {
        for (const PlayerBody3D& body : impl.grid) {
            impl.bodyBundles.push_back(AthleteBundle3D::shared(settings.assetsDirectory, body));
            impl.bodyContracts.push_back(std::make_unique<LocomotionContract3D>(impl.bodyBundles.back()->profile(), &body));
            if (impl.bodyContracts.back()->hash() != impl.contract->hash())
                throw std::runtime_error("treino da locomocao: o contrato de um corpo da grade difere do de hoje");
        }
        if (settings.evaluation && impl.task.body.evaluationGrid)
            impl.evaluationBodies = evaluationBodies(impl.grid, impl.task.body);
    }
    impl.observationSize = impl.contract->observationSize();
    impl.actionSize = impl.contract->actionSize();
    impl.drillCount = impl.task.drills.size();
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
    batchSettings.world.groundHalfExtentMeters = impl.task.groundHalfExtentMeters;
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
const std::vector<PlayerBody3D>& LocomotionTraining3D::bodyGrid() const { return m_impl->grid; }
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

std::vector<float> LocomotionTraining3D::actionScales() const {
    std::vector<float> out(m_impl->actionSize, 0.0f);
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = m_impl->contract->channelScale(i, true);
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

LocomotionTraining3D::ActionBounds LocomotionTraining3D::actionBounds() const {
    return { m_impl->boundLower, m_impl->boundUpper };
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
    // (Os corpos mudam o contrato e os pacotes: outro lote.)
    if (!(task.body == impl.task.body))
        throw std::runtime_error("treino da locomocao: a tarefa nova muda os corpos (abra outro lote)");
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

std::size_t LocomotionTraining3D::savedStartCount(std::size_t drill) const {
    return drill < m_impl->savedBank.size() ? m_impl->savedBank[drill].size() : 0;
}

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
    impl.bankRng.seed(seed * 0xD1B54A32D192ED03ull + 7);
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) {
        impl.worlds[i].rng.seed(seed * 0x9E3779B97F4A7C15ull + i * 0xBF58476D1CE4E5B9ull + 1);
        // (O sorteio do corpo, a parte: so com o bloco ele e usado.)
        impl.worlds[i].bodyRng.seed(seed * 0x94D049BB133111EBull + i * 0xD6E8FEB86659FD93ull + 3);
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
    // Os comecos guardados aceitos nesta decisao, na ordem dos mundos (o banco
    // cheio troca um ao acaso, pelo sorteio do lote: reproduzivel).
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) {
        Impl::World& world = impl.worlds[i];
        if (!world.hasAccepted) continue;
        world.hasAccepted = false;
        const std::size_t drill = world.accepted.drill;
        if (drill >= impl.savedBank.size()) continue;
        std::vector<Impl::SavedStart>& bank = impl.savedBank[drill];
        if (bank.size() < impl.task.savedStarts.capacity) bank.push_back(std::move(world.accepted));
        else bank[std::uniform_int_distribution<std::size_t>(0, bank.size() - 1)(impl.bankRng)] = std::move(world.accepted);
    }
    impl.lastStepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

std::string LocomotionTraining3D::frameLines(std::size_t environment) const {
    const Impl& impl = *m_impl;
    if (environment >= impl.worlds.size()) return {};
    const Impl::World& world = impl.worlds[environment];
    const RagdollState3D& body = impl.batch->world(environment).body();
    const RagdollProfile3D& profile = impl.bundleOf(world).profile();
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
