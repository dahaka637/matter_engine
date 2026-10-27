#pragma once

// O que a sonda e os seus modos da L0 (a paridade, a linha de base:
// ProbeL0.cpp) dividem: o roteiro (as teclas, os empurroes, as caixas), as
// medidas, o contexto e os ganchos do contrato da rede.
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Athlete/Training/LocomotionWorld3D.hpp"
#include "Engine/Athlete/Reference/BodyForecast3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine::Probe {

inline constexpr float Pi = 3.14159265358979323846f;
inline constexpr float Dt = 1.0f / 120.0f;

// O que o "jogador" faz num instante: teclas no referencial da camera (x
// frente, y esquerda), o olhar e os pedidos.
struct Keys {
    float forward = 0.0f;
    float left = 0.0f;
    bool sprint = false;
    // O sprint maximo (o Shift duplo do laboratorio: so no padrao, com sprint).
    bool maxSprint = false;
    float cameraYaw = 0.0f;
    bool jump = false;
    bool crouch = false;
    BallActionKind3D ball = BallActionKind3D::None;
    float power = 0.5f;
};

// Empurrao: forca (N, no mundo) por um tempo, repartida em pelve, peito e
// alto do peito como o teste de impacto do laboratorio.
struct Push {
    float at = 0.0f;
    float seconds = 0.1f;
    Vec3 force;
};

// Uma caixa parada no mundo (parede, degrau, rampa): centro, meia-dimensao e
// orientacao.
struct StaticBox {
    Vec3 center;
    Vec3 half;
    Quaternion orientation;
};

// Um objeto arremessado no atleta (a pancada de verdade: contato, nao
// forca): caixa de massa e meia-dimensao dadas, saindo de `start` (em
// relacao aos pes) com `velocity` no instante `at`.
struct Throw {
    float at = 0.0f;
    float massKg = 20.0f;
    Vec3 halfExtents { 0.2f, 0.2f, 0.2f };
    Vec3 start;
    Vec3 velocity;
};

struct Scenario {
    std::string name;
    float seconds;
    std::function<Keys(float)> keys;
    bool ball = false;
    Vec3 ballStart;     // em relacao aos pes, no referencial inicial
    Vec3 ballVelocity;
    std::vector<Push> pushes;
    std::vector<Throw> throws;
    // Pedido automatico com a bola a menos disto da pelve (no plano).
    float autoDistance = 0.0f;
    BallActionKind3D autoKind = BallActionKind3D::Dribble;
    // O rumo pedido vira isto (rad, + esquerda) a partir do pedido automatico.
    float autoTurn = 0.0f;
    // Conduzir (o jogador que toca de novo sempre que a bola chega perto): o
    // intervalo minimo entre os pedidos automaticos (s) e a forca deles (<0:
    // a padrao do tipo).
    float autoCooldown = 10.0f;
    float autoPower = -1.0f;
    // Manejo v2: a bola posta em relacao a pelve no instante placeAt (x a
    // frente, y a esquerda no rumo do corpo; z a altura do centro), andando
    // junto com o corpo; o pedido sai na mesma hora (pelas teclas). E o
    // instante em que o movimento pedido muda depois dele (-1 nenhum).
    float placeAt = -1.0f;
    Vec3 placeOffset;
    // A bola posta ja vindo (a recepcao): a velocidade dela no mundo, no
    // referencial do corpo (zero: a da capsula, andando junto).
    Vec3 placeVelocity;
    // Mais bolas postas do mesmo jeito (o teste sorteado): o instante e o lugar.
    std::vector<std::pair<float, Vec3>> placements;
    float changeAt = -1.0f;
    // Comeca no fisico (o empurrao so age no corpo fisico).
    bool startPhysical = false;
    // Uma parede parada (centro e meia-dimensao; sem parede, meia-dimensao 0)
    // e o terreno parado (degraus, escada, rampa).
    Vec3 wallCenter;
    Vec3 wallHalf;
    std::vector<StaticBox> statics;
};

struct Metrics {
    std::vector<float> footSlide;
    float peakUpperAcceleration = 0.0f;
    float peakLegAcceleration = 0.0f;
    float worstPoseJump = 0.0f;
    int modeSwitches = 0;
    float animatedSeconds = 0.0f;
    float physicalSeconds = 0.0f;
    float returningSeconds = 0.0f;
    float activeSeconds = 0.0f;
    float assistImpulse = 0.0f;
    int requests = 0;
    int touches = 0;
    bool fell = false;
    float lowestPelvis = 10.0f;
    float ballSpeedAfter = 0.0f;
    // O pe apoiado (o estilo diz apoio, no modo padrao): a velocidade dele
    // no plano (m/s) - zero e o certo. Pegadas: a maior correcao na saida do
    // apoio (m), o maior erro de alcance da perna (m) e os passos de
    // acomodacao.
    std::vector<float> plantedSlide;
    float worstLiftCorrection = 0.0f;
    float worstReach = 0.0f;
    int settleSteps = 0;
    int earlyLifts = 0;
    int catchSteps = 0;
    int startSteps = 0;
    // Os modos por onde passou ("animado>ativo>volta>animado") e o do fim.
    std::string trail;
    // O joelho: a maior velocidade de esticar (graus/s) no modo padrao, no
    // corpo e no alvo (a pose), e a maior aceleracao angular dele (graus/s2)
    // no corpo - o estalo da perna que estica de uma vez.
    float peakKneeExtension = 0.0f;
    float peakKneeExtensionTarget = 0.0f;
    float peakKneeAcceleration = 0.0f;
    // A bola: o primeiro pedido e a primeira forca (s; -1 nao houve) e o
    // caminho da capsula; e os ticks com cada perna num gesto (a animacao de
    // fato tocando).
    float requestTime = -1.0f;
    float forceTime = -1.0f;
    // A base mais alta das areas de alcance dos pes (m; sobe com o pulo).
    float reachBaseMax = 0.0f;
    std::array<int, 2> gestureTicks {};
    // Ticks com as duas pernas em gesto (nenhum pe de apoio) e a sequencia dos
    // gestos (a perna e o instante de cada um).
    int bothGestureTicks = 0;
    std::string gestureSequence;
    // Os pes parados (no plano, em relacao a pelve, no rumo) no comeco e no
    // fim: o corpo fisico e a animacao (o "perna deslocada" depois do gesto).
    bool stanceValid = false;
    std::array<Vec3, 2> stanceStart {}, stanceEnd {}, animatedStart {}, animatedEnd {};
    // Parado de verdade (sem tecla, sem giro da camera, sem gesto) ha mais de
    // 1 s: o pior pe fora do lugar e quando.
    float stillWorst = 0.0f, stillWorstTime = -1.0f, stillSeconds = 0.0f;
    std::array<Vec3, 2> stillWorstFeet {};
    std::vector<Vec3> capsulePath;
    // O levantar: no comeco (o primeiro Rising) a frente do peito e a da
    // pelve (z: + de costas, - de brucos) e o lado escolhido; quando acabou
    // de pe (s; -1 nao levantou).
    // Conduzir: a bola longe do corpo (mais de 2,5 m no plano) por quanto
    // tempo (s) e a distancia media dela (m).
    float ballFarSeconds = 0.0f;
    float ballDistanceSum = 0.0f;
    int ballDistanceCount = 0;
    // O terreno: a maior velocidade para cima da capsula e da pelve (m/s) - o
    // lancamento - e a altura maxima da pelve sobre a capsula (m).
    float capsuleUp = 0.0f;
    float pelvisUp = 0.0f;
    float pelvisOverCapsule = 0.0f;
    float riseAt = -1.0f;
    float riseChest = 0.0f;
    float risePelvis = 0.0f;
    int riseSide = 0;
    float risenAt = -1.0f;
    // O quanto a pelve andou no plano (m), e as idas ao fisico depois de ja
    // ter voltado uma vez ao padrao (a recaida).
    float travel = 0.0f;
    int relapses = 0;
    BodyModeKind3D finalMode = BodyModeKind3D::Animated;
    // Pegadas novas (plano v3): as pernas cruzadas (ticks com um tornozelo do
    // lado errado do outro, lado a lado, no rumo da pelve) e a maior
    // sobreposicao entre as pernas (m, as capsulas e as caixas dos pes, acima
    // da do parado); o pe apoiado tremendo (as trocas de sentido da
    // velocidade vertical, no alvo e no corpo); o pe travado longe do chao
    // (m, a sola contra o chao embaixo); o pe no ar longe do estilo (m,
    // depois de 0,15 s no ar).
    int legCrossTicks = 0;
    // Sem o limite de frente: o pe ou o joelho esquerdo a direita do outro (21/10).
    int legOrderTicks = 0;
    int kneeOrderTicks = 0;
    float legOverlap = 0.0f;
    // A troca fisico -> padrao: a maior diferenca de velocidade pelve x
    // capsula e o maior pico de aceleracao dos links nela.
    float transitionSpeedGap = 0.0f;
    float transitionPeak = 0.0f;
    // E a pelve mais rapida nela e o quanto o peito inclinou em 0,1 s.
    float transitionSpeedMax = 0.0f;
    float transitionTrunkRate = 0.0f;
    // O atraso do clique a forca (s), a "perna louca" (ticks com coxa ou
    // canela girando a mais de 30 rad/s; e com o joelho virando de lado mais
    // de 60 graus num tick) e o maior voo (os dois pes no ar, s).
    int lostRequests = 0;
    // Ticks com um pe entrando mais de 2 cm na bola.
    int footOnBallTicks = 0;
    std::vector<float> latencies;
    std::vector<float> pendingRequests;
    int crazyLegTicks = 0;
    int kneeFlipTicks = 0;
    float legSpinMax = 0.0f;
    float longestFlight = 0.0f;
    float flightRun = 0.0f;
    // Plano v5 (E1): a previsao do pouso de cada pe (feita 0,2-0,3 s antes)
    // contra o pouso de verdade: o erro do instante (s) e da posicao (m, no
    // plano).
    std::array<float, 2> forecastAt { -1.0f, -1.0f };
    std::array<Vec3, 2> forecastFoot {};
    std::array<bool, 2> forecastWasPlanted { true, true };
    std::vector<float> forecastTimeErrors;
    std::vector<float> forecastPositionErrors;
    // Plano v5 (E2): a capsula prevista 0,3 s antes contra a de verdade (m).
    std::vector<std::pair<float, Vec3>> bodyForecasts;
    std::vector<float> bodyForecastErrors;
    float lastFacingYaw = 0.0f;
    int stanceTremor = 0;
    int bodyTremor = 0;
    float lockGap = 0.0f;
    float airPull = 0.0f;
};

struct ProbeContext {
    const RagdollProfile3D* profile = nullptr;
    CharacterGaitSpeeds3D gaitSpeeds;
    bool noCapsule = false;
    CharacterMotorSettings3D avatar;
    AthleteRuntimeAssets3D runtimeAssets;
    // O corpo do atleta (--corpo; sem a opcao, o de hoje).
    PlayerBody3D body;
};

// A paridade do contrato da rede (o degrau 1): a acao fixa de cada decisao
// (deterministica), aplicada com a rampa, e a impressao da entrada da rede a
// cada tick.
struct ContractHooks {
    const LocomotionContract3D* contract = nullptr;
    bool act = false;
    LocomotionAuthority3D authority;
    std::vector<std::uint64_t>* observations = nullptr;
    std::size_t nonFinite = 0;
    BodyForecast3D forecast;
    std::vector<float> buffer;
};

// O roteiro inteiro (sonda).
std::vector<Scenario> scenarios();
// Um cenario do comeco ao fim (trace: a impressao de cada tick; hooks: o
// contrato da rede).
Metrics runScenario(const ProbeContext& context, const Scenario& scenario, const AthleteRuntimeSettings3D& settings,
    bool startPhysicalArgument, FILE* dump, int every, bool suppressRequests = false,
    std::vector<AthleteTickDigest3D>* trace = nullptr, ContractHooks* hooks = nullptr);

// Os ganchos do contrato (ProbeL0.cpp).
std::vector<float> parityAction(const LocomotionContract3D& contract, int decision);
std::uint64_t observationPrint(std::span<const float> values);
void contractBefore(ContractHooks* hooks, AthleteState3D& athlete, int step);
void contractAfter(ContractHooks* hooks, const AthleteState3D& athlete, const RagdollState3D& body,
    const AthleteCommand3D& command);

// Os modos da L0 (ProbeL0.cpp).
AthleteCommand3D scenarioCommand(const ProbeContext& context, const AthleteRuntimeSettings3D& settings, const Keys& keys,
    float terrainSpeedLimit);
void stepScenarioWorld(const ProbeContext& context, const AthleteRuntimeSettings3D& settings, const Scenario& scenario,
    LocomotionWorld3D& world, int step, ContractHooks* hooks = nullptr);
std::vector<LocomotionStaticBox3D> scenarioStatics(const Scenario& scenario);
int runParity(const ProbeContext& context, int limit, int restores);
int runBaseline(const ProbeContext& context, int threads);
// A prova da acao zero do contrato com autoridade plena (plano RL sobre o
// classico, ordem 4): por roteiro e nivel de ajuda, o classico sem rede contra
// o contrato decidindo zero com as cabecas todas ligadas, bit a bit por tick.
int runZeroProof(const ProbeContext& context, int threads);

} // namespace MatterEngine::Probe
