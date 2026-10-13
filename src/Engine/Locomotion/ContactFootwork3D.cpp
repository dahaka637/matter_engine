#include "Engine/Locomotion/ContactFootwork3D.hpp"

#include "Engine/Character/CharacterFootGeometry3D.hpp"
#include "Engine/Character/CharacterStateEstimator3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265358979323846f;

// Passo de acomodacao: pe apoiado a mais de 12 cm (ou 34 graus) de onde a
// pose o quer.
constexpr float SettleStepDistance = 0.12f;
constexpr float SettleStepYaw = 0.60f;
// Giro parado: perna torcida demais (24 graus), ou a base ainda mais de 24
// graus atras do alvo (a camera continuou girando durante o primeiro par:
// esperar nova torcao deixava a pelve presa na orientacao intermediaria).
// (Era 24 graus: o giro parado demorava a comecar e a pelve empacava no
// limite de torcao do quadril esperando os pes.)
constexpr float TurnTwistTrigger = 0.21f;
constexpr float TurnEarlyRemaining = 0.21f;
// TEMPTUNE: com a ajuda cheia, abaixo desta torcao (rad) a base se acerta por
// acomodacao, nao por giro.
const float HeldTurnTwist = [] { const char* v = std::getenv("T_HeldTurnTwist"); return v ? static_cast<float>(std::atof(v)) : 0.6f; }();
// Passo de giro: descarga curta e balanco rapido (o pe so abre sobre a bola
// e gira); uma recuperacao leve (urgencia abaixo disto) nao cancela o giro -
// o proprio passo de giro refaz a base.
constexpr float TurnUnloadSeconds = 0.12f;
constexpr float TurnRecoveryUrgency = 0.25f;
// Ate 72 graus por par de passos: o pe pousa girado em relacao a pelve, que
// ainda esta a caminho, e o quadril so gira 45 graus. Mirando 115 de uma vez
// (meia-volta), o pe pousava onde a perna nao chega.
const float TurnStepLimit = [] { const char* v = std::getenv("T_TurnStepLimit"); return v ? static_cast<float>(std::atof(v)) : 1.25f; }();
// TEMPTUNE: base compacta do giro parado (ajuda cheia) - meia largura (m, do
// centro ao tornozelo), o esquerdo um pouco a frente (m) e quanto cada pe
// abre para fora (rad); 0 na largura = a pose parada girada. O modo vale com
// o giro pedido acima de TurnModeEnter (rad) ou a pelve girando acima de
// TurnModeRate (rad/s; a 0,6 a pose de alerta entrando depois de uma freada -
// a pelve roda 22 graus - ligava o modo sem giro nenhum), e TurnModeHold (s)
// depois.
const float TurnStanceHalfWidth = [] { const char* v = std::getenv("T_TurnHalfWidth"); return v ? static_cast<float>(std::atof(v)) : 0.0f; }();
const float TurnStanceStagger = [] { const char* v = std::getenv("T_TurnStagger"); return v ? static_cast<float>(std::atof(v)) : 0.0f; }();
const float TurnStanceToeOut = [] { const char* v = std::getenv("T_TurnToeOut"); return v ? static_cast<float>(std::atof(v)) : 0.15f; }();
const float TurnModeEnter = [] { const char* v = std::getenv("T_TurnModeEnter"); return v ? static_cast<float>(std::atof(v)) : 0.20f; }();
const float TurnModeRate = [] { const char* v = std::getenv("T_TurnModeRate"); return v ? static_cast<float>(std::atof(v)) : 1.5f; }();
const float TurnModeHold = [] { const char* v = std::getenv("T_TurnModeHold"); return v ? static_cast<float>(std::atof(v)) : 0.5f; }();
const float TurnStanceScale = [] { const char* v = std::getenv("T_TurnStanceScale"); return v ? static_cast<float>(std::atof(v)) : 1.0f; }();
const float HeldTurnStepLimit = [] { const char* v = std::getenv("T_HeldTurnStep"); return v ? static_cast<float>(std::atof(v)) : 1.6f; }();
const float TurnModeDrift = [] { const char* v = std::getenv("T_TurnModeDrift"); return v ? static_cast<float>(std::atof(v)) : 0.25f; }();
// TEMPTUNE: passos de giro com a ajuda cheia (o equilibrio e dela): sai na
// hora (fracao do tempo de descarga) e o balanco curto, mais curto quanto mais
// rapido o giro (s: o mais lento e o mais rapido).
const float HeldTurnUnloadShare = [] { const char* v = std::getenv("T_HeldTurnUnload"); return v ? static_cast<float>(std::atof(v)) : 0.15f; }();
const float HeldTurnSwingSlow = [] { const char* v = std::getenv("T_HeldTurnSwingSlow"); return v ? static_cast<float>(std::atof(v)) : 0.17f; }();
const float HeldTurnSwingFast = [] { const char* v = std::getenv("T_HeldTurnSwingFast"); return v ? static_cast<float>(std::atof(v)) : 0.10f; }();
// Recuperacao: o ponto de captura fora do apoio planejado alem disto, e o
// corpo indo para fora (ou sob forca externa).
constexpr float RecoveryOutsideTrigger = 0.045f;
// TEMPTUNE: com a ajuda cheia segurando, quanto a mais a captura precisa
// sair da base (m) para um passo de recuperacao sem forca de fora.
const float AssistHoldMargin = [] { const char* v = std::getenv("T_AssistHoldMargin"); return v ? static_cast<float>(std::atof(v)) : 2.0f; }();
constexpr float RecoveryOutsideFull = 0.20f;
constexpr float RecoveryLeavingSpeed = 0.10f;
constexpr float RecoveryForceTrigger = 60.0f;
constexpr float RecoveryCaptureUnloadSeconds = 0.15f;
constexpr float CaptureTargetSlowSpeed = 0.30f;
constexpr float CaptureTargetFastSpeed = 0.60f;
constexpr float RecoveryMemorySeconds = 0.45f;
// Girando parado sem forca de fora: a captura pode sair isto a mais da base
// antes de virar recuperacao (m). TEMPTUNE
const float TurnRecoveryMargin = [] { const char* v = std::getenv("T_TurnRecoveryMargin"); return v ? static_cast<float>(std::atof(v)) : 0.15f; }();
// Descarga antes de soltar o pe: sai quando a carga medida dele cai abaixo
// da fracao, ou no tempo maximo (passo comum), ou logo na emergencia.
constexpr float UnloadedShare = 0.30f;
// Descarga parcial: depois de tentar 0,2 s, abaixo disto ja sai (um passo
// rapido de gente tambem sai com o pe ainda levemente carregado).
constexpr float PartialUnloadShare = 0.42f;
constexpr float PartialUnloadSeconds = 0.20f;
constexpr float UnloadMaximumSeconds = 0.30f;
// Passo comum em que o pe nunca saiu do chao (foi arrastado: a carga nao
// passou): o proximo passo comum desse pe espera, e cada vez mais.
constexpr float DraggedCooldownSeconds = 0.60f;
constexpr float DraggedCooldownMaximumSeconds = 4.80f;
constexpr float RecoveryUnloadMaximumSeconds = 0.06f;
// Pouso: contato real depois de metade do passo (pe quase parado no plano,
// ou ja no fim), e carga ate apoio.
constexpr float EarlyTouchdownProgress = 0.5f;
constexpr float LateTouchdownProgress = 0.75f;
constexpr float TouchdownPlanarSpeed = 0.5f;
constexpr float LoadedShare = 0.15f;
constexpr float LoadingMaximumSeconds = 0.05f;
// E5: carga aceita (apoio continuo e sem escorregar) e o teto dela.
constexpr float LoadAcceptanceSeconds = 0.025f;
constexpr float LoadAcceptanceSlipSpeed = 0.5f;
constexpr float LoadAcceptanceLimitSeconds = 0.40f;
// Procurando o chao depois do tempo do passo: desce ate 30 cm, a 0,3 m/s.
constexpr float SearchDescentSpeed = 0.30f;
constexpr float SearchDescentLimit = 0.30f;
// Esbarrou em algo no caminho: o arco sobe isto a mais.
constexpr float BlockedExtraClearance = 0.06f;

// Passos de freada (o planejador dono dos pes enquanto o corpo freia):
// calibrados pela varredura de freada (MATTERENGINE_TEST_FILTER=sweepbrake)
// e medidos contra Run To Stop / Running To Stop (pe de freada 0,43-0,60 m a
// frente do centro de massa, sem voo, base final escalonada ate ~0,45 m).
// Balanco (s): o tempo de um passo de freada (em qualquer velocidade dos
// passos: a corrida para acima de ~2,2 m/s); assumindo um pe ja no ar, uma
// fracao dele.
constexpr float BrakeSwingSeconds = 0.20f;
constexpr float BrakeTakeoverSwingShare = 0.8f;
// Distancia do pe de freada a frente do centro de massa no toque: d = h a /
// g pela desaceleracao (m/s2), entre o minimo e o alcance de cada rumo em
// relacao ao corpo (m) - a frente o quadril flexiona muito, de lado e para
// tras bem menos.
constexpr float BrakePlaceDeceleration = 5.5f;
constexpr float BrakePlaceMinimum = 0.15f;
constexpr float BrakePlaceForward = 0.55f;
constexpr float BrakePlaceBackward = 0.32f;
constexpr float BrakePlaceSideways = 0.40f;
// Passos de freada por freada, no maximo (o pedido do usuario: freada
// imediata, um passo se der, dois se precisar; depois o equilibrio de parado
// assume).
constexpr int BrakeMaximumSteps = 2;
// O pouso devagar fica isto atras do ponto de captura (m), andando para a
// frente/tras e de lado.
constexpr float BrakeStraddle = 0.06f;
constexpr float BrakeStraddleSideways = 0.12f;
// Base: meia largura de cada pe ao lado da trajetoria, a largura de quadril
// do passo que fecha de lado, e o alcance a partir do pe de apoio (m).
constexpr float BrakeHalfWidth = 0.11f;
constexpr float BrakeHipWidth = 0.22f;
constexpr float BrakeReach = 1.0f;
// Arco do balanco (m): sobe com a velocidade (o pe vem de longe atras do
// corpo e raspava no chao).
constexpr float BrakeLift = 0.07f;
constexpr float BrakeLiftPerSpeed = 0.02f;
constexpr float BrakeLiftMaximum = 0.20f;
// Tocando o chao a frente do centro de massa (m) depois de metade do passo,
// pousa ali.
constexpr float BrakeTouchAhead = 0.10f;
// Quando sai um passo: o ponto de captura alem do pe da frente (m), ou a
// base comprida demais ja devagar (m) - so se o ponto de captura nao saiu
// dela para o lado: entao a freada entrega para a recuperacao; de lado
// (rumo a mais de ~45 graus do corpo), o pe de dentro fecha alem desta
// largura (m).
constexpr float BrakeStepTrigger = 0.08f;
constexpr float BrakeCloseStance = 0.75f;
constexpr float BrakeLateral = 0.70f;
constexpr float BrakeSideClose = 0.38f;
// Pe assumido da passada que ja estava no chao alem disto atras do centro
// de massa (m), com o outro no ar: pousa atras do corpo (m), como o apoio de
// tras do afundo.
constexpr float BrakeRearPlantAhead = 0.10f;
constexpr float BrakeRearBehind = 0.15f;
// Parando de uma freada, o pe ate isto acima do chao (m) conta como apoio na
// decisao de recuperacao.
constexpr float RecoveryNearGround = 0.03f;
// O ponto de captura mais que isto (m) atras do pe de tras: a freada nao
// terminou - um passo curto para tras.
constexpr float BrakeSettleMargin = 0.03f;
// Pe apoiado deslizando mais que isto (m/s): a ancora vai junto.
constexpr float BrakeSlideSpeed = 0.3f;
inline float tuneTempFootwork(const char* name, float fallback) {  // TEMPTUNE
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}
// Assumindo os dois pes no ar freando, so o mais alto da passo de freada.
const bool BrakeSingleTakeoverStep = tuneTempFootwork("T_BrakeSingle", 1.0f) > 0.5f;  // TEMPTUNE
// Recem-freado, o pe fora do alcance (sem apoio) volta para a base.
const bool UnreachableDuringSettleHold = tuneTempFootwork("T_UnreachHold", 1.0f) > 0.5f;  // TEMPTUNE
// Freando de lado, o pe de dentro assumido no ar so pousa (o de fora abre).
const bool SidewaysInnerLands = tuneTempFootwork("T_SideInner", 0.0f) > 0.5f;  // TEMPTUNE
// Freando de lado: o ponto de captura alem disto (m) do pe de fora, ele abre
// antes de o de dentro fechar.
const bool SideOpenFirst = tuneTempFootwork("T_SideOpen", 0.0f) > 0.5f;  // TEMPTUNE
const float SideOpenFirstBeyond = tuneTempFootwork("T_SideOpenBeyond", 0.20f);  // TEMPTUNE
// Recuperacao: so o pe escolhido muda de passo no ar.
const bool RecoverOnlyChosenFoot = tuneTempFootwork("T_RecoverChosen", 0.0f) > 0.5f;  // TEMPTUNE
// Recem-freado, base mais estreita que isto (m; negativa = cruzada) acomoda.
const float SettleHoldNarrowBase = tuneTempFootwork("T_NarrowBase", 0.06f);  // TEMPTUNE

// Normal que pode inclinar a sola: piso (ate ~25 graus). Uma quina de degrau
// devolve a normal da aresta (medido: 40 graus) - pe nenhum pousa assim.
Vec3 soleNormal(const FootSurface3D& surface) {
    return surface.valid && surface.normal.z >= 0.90f ? surface.normal
        : Vec3 { 0.0f, 0.0f, 1.0f };
}

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float wrapAngle(float radians) {
    radians = std::fmod(radians + Pi, 2.0f * Pi);
    if (radians < 0.0f) radians += 2.0f * Pi;
    return radians - Pi;
}

// Pendulo invertido linear no plano: o corpo (COM, velocidade) depois de t
// segundos com o centro de pressao parado em cop (forma fechada).
struct PlanarBody3D {
    Vec3 com;
    Vec3 velocity;
};

// O corpo pela tarefa corporal (fase 11: a aceleracao alocada, nao o
// pendulo passivo): ele acelera para a velocidade pedida (ate 4 m/s^2, em
// ~0,25 s) com o centro de pressao onde isso pede, limitado ao apoio - a
// sola (6 cm em volta do centro) ou o segmento entre as duas solas.
constexpr float PredictionStepSeconds = 0.01f;
constexpr float PredictionSoleRadius = 0.06f;
constexpr float PredictionAcceleration = 4.0f;
constexpr float PredictionResponseSeconds = 0.25f;
// A dupla sustentacao da agenda (toque, carga aceita, descarga do outro).
// (A medida - de lado o pe da frente levava ate 0,29 s, pelo prazo - fazia o
// pouso previsto abrir o pe da frente 14 cm alem do ponto de captura: o corpo
// freava de 0,6 a 0,05 m/s a cada passo.)
constexpr float PredictionDoubleSupportSeconds = 0.06f;

PlanarBody3D supportAdvance(PlanarBody3D body, Vec3 from, Vec3 to, Vec3 desired, float omega, float seconds) {
    const int steps = std::max(1, static_cast<int>(std::ceil(seconds / PredictionStepSeconds)));
    const float h = seconds / static_cast<float>(steps);
    const Vec3 segment = to - from;
    const float length = segment.lengthSquared();
    for (int step = 0; step < steps; ++step) {
        Vec3 want = (desired - body.velocity) * (1.0f / PredictionResponseSeconds);
        const float wanted = want.length();
        if (wanted > PredictionAcceleration) want = want * (PredictionAcceleration / wanted);
        Vec3 cop = body.com - want * (1.0f / (omega * omega));
        const float along = length > 1e-6f ? std::clamp(dot(cop - from, segment) / length, 0.0f, 1.0f) : 0.0f;
        const Vec3 nearest = from + segment * along;
        Vec3 away = cop - nearest;
        const float distance = away.length();
        if (distance > PredictionSoleRadius) away = away * (PredictionSoleRadius / distance);
        cop = nearest + away;
        body.velocity += (body.com - cop) * (omega * omega * h);
        body.com += body.velocity * h;
    }
    return body;
}

Vec3 clampMagnitude(Vec3 value, float maximum) {
    const float squared = value.lengthSquared();
    if (squared <= maximum * maximum || squared < 0.000001f) return value;
    return value * (maximum / std::sqrt(squared));
}

bool supportsWeight(FootPhase3D phase) {
    return phase == FootPhase3D::Stance || phase == FootPhase3D::Unloading
        || phase == FootPhase3D::Loading;
}

bool airborne(FootPhase3D phase) {
    return phase == FootPhase3D::Swing || phase == FootPhase3D::TouchdownSearch;
}

// Contato de apoio de verdade neste tick: evento com impulso e normal para
// cima. Memoria (evento que piscou) e evidencia geometrica nao pousam pe.
bool contactSupport(const FootSupportEstimate3D& foot) {
    return foot.supporting && !foot.heldFromMemory
        && (foot.evidence == ContactEvidence3D::EstimatedImpulse
            || foot.evidence == ContactEvidence3D::SolvedImpulse);
}

// Duracao de um passo comum pela distancia e pelo giro do pe: na base lado a
// lado um giro de 90 graus anda ~15 cm por pe (0,33 s); na base em alerta o
// pe anda ~40 cm (0,42 s; nas referencias, ~0,45 s).
float ordinaryStepSeconds(float distance, float yaw) {
    return std::clamp(0.22f + 0.35f * distance + 0.06f * yaw / (0.5f * Pi), 0.24f, 0.48f);
}

float turnStepSeconds(float distance, float yaw) {
    return std::clamp(0.16f + 0.24f * distance + 0.04f * yaw / (0.5f * Pi), 0.18f, 0.29f);
}

float recoveryStepSeconds(float urgency) {
    return 0.30f - 0.12f * std::clamp(urgency, 0.0f, 1.0f);
}

// Locomocao pelo planejador (E5): balanco do passo pela velocidade (mais
// rapido, passo mais curto no tempo) e a dupla sustentacao entre passos.
float locomotionSwingSeconds(float speed) {
    return std::clamp(0.34f - 0.035f * speed, 0.20f, 0.34f);
}
constexpr float LocomotionDoubleSupportSeconds = 0.04f;
constexpr float LocomotionUnloadSeconds = 0.30f;
constexpr float LocomotionMinimumSpeed = 0.15f;
constexpr float LocomotionSpeedChangePerStep = 1.2f;
// Marcha: fracao do caminho do DCM (sola que sai -> sola que recebe) que
// conta como apoio transferido.
constexpr float LocomotionTransferDone = 0.60f;
constexpr float LocomotionStepWidth = 0.20f;
// Balanco da marcha (E5, fase 10): o alvo do pouso muda livre ate 70% do
// balanco e depois fica. (A politica do pacote - livre ate 35%, no maximo
// 10 cm em volta do alvo de 35% ate 60%, depois fixo - foi medida: 1,5 m/s,
// 6 -> 9 quedas em 16 e o dobro de ticks com os dois pes no ar; acelerando,
// o pouso precisa das revisoes tardias. A janela limitada fica para quando o
// pouso previsto (fase 11) revisar menos.)
constexpr float SwingFreeRevisionProgress = 0.70f;
constexpr float SwingBoundedRevisionProgress = 0.70f;
constexpr float SwingBoundedRevisionMeters = 0.10f;
constexpr float SwingRefusedRevisionMeters = 0.03f;

// Um tick (dt) na quintica que leva (p, v, a) ao alvo, parado, com a
// aceleracao final aEnd, em T segundos. Refeita a cada tick do ponto em que
// ja esta, ela e a mesma curva enquanto o alvo nao muda; se muda, a curva
// nova sai do mesmo ponto, velocidade e aceleracao. (Com a aceleracao
// inicial e final da cubica suave - +-6 D / T^2 -, sem revisao ela E a
// cubica do balanco de antes.)
void advanceQuintic(Vec3& p, Vec3& v, Vec3& a, Vec3 target, Vec3 aEnd, float T, float dt) {
    if (T <= dt) {
        p = target;
        v = {};
        a = {};
        return;
    }
    const Vec3 d = target - p;
    const float T2 = T * T, T3 = T2 * T;
    const Vec3 c2 = a * 0.5f;
    const Vec3 c3 = (d * 20.0f - v * (12.0f * T) - (a * 3.0f - aEnd) * T2) * (1.0f / (2.0f * T3));
    const Vec3 c4 = (d * -30.0f + v * (16.0f * T) + (a * 3.0f - aEnd * 2.0f) * T2) * (1.0f / (2.0f * T3 * T));
    const Vec3 c5 = (d * 12.0f - v * (6.0f * T) + (aEnd - a) * T2) * (1.0f / (2.0f * T3 * T2));
    const float t2 = dt * dt, t3 = t2 * dt;
    const Vec3 p0 = p, v0 = v;
    p = p0 + v0 * dt + c2 * t2 + c3 * t3 + c4 * (t3 * dt) + c5 * (t3 * t2);
    v = v0 + c2 * (2.0f * dt) + c3 * (3.0f * t2) + c4 * (4.0f * t3) + c5 * (5.0f * t3 * dt);
    a = c2 * 2.0f + c3 * (6.0f * dt) + c4 * (12.0f * t2) + c5 * (20.0f * t3);
}

// Quanto do caminho da pelve ate o pe de apoio o peso anda na descarga, e o
// limite (m): lado a lado, um quarto do caminho (ate 5 cm); na base
// escalonada o pe anda 40 cm e o peso vai quase inteiro para o outro. Entra
// em 0,2 s: em 0,1 s a pelve ia 14 cm a ~1,4 m/s e o ponto de captura saia
// do apoio (o corpo "caia" para o pe de apoio). (Crescer isso ate a carga
// passar, sem a assistencia, so derrubava o corpo para o lado - a troca de
// peso de verdade e da atuacao das juntas, etapa E4.)
// Pe roteirizado com peso no apoio: ate onde a pelve vai (m).
constexpr float ScriptedShiftLimit = 0.16f;

void unloadShift(float seconds, bool staggered, float& fraction, float& limit) {
    const float ramp = smoothStep(seconds / 0.20f);
    fraction = ramp * (staggered ? 0.45f : 0.25f);
    limit = staggered ? 0.14f : 0.05f;
}

} // namespace

void ContactFootwork3D::reset(const RagdollProfile3D& profile) {
    m_plan = {};
    m_linkCount = profile.links.size();
    m_feet = { m_linkCount, m_linkCount };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") m_feet[0] = i;
        if (profile.links[i].id == "RightFoot") m_feet[1] = i;
    }
    m_nextStepId = 1;
    m_pendingTurnFoot = -1;
    m_sinceRecoveryNeeded = 10.0f;
    m_recoveryTarget = {};
    m_releaseShiftFraction = {};
    m_releaseShiftLimit = {};
    m_stepCooldown = {};
    m_draggedBackoff = {};
    m_ticksWithoutContact = {};
    m_liftedOff = {};
    m_residualShift = {};
    m_fitCenter = {};
    m_fitShift = {};
    m_fitValid = {};
    m_turnSign = { 1.0f, 1.0f };
    m_lastRecoverySide = -1;
    m_lastLocomotionSide = -1;
    m_sinceLocomotionTouchdown = 10.0f;
    m_locomotionPeriod = 0.45f;
    m_benchHolding = {};
    m_brakeClosing = {};
    m_brakeRearLanding = {};
    m_brakeStepsTaken = 0;
    m_stumbleStep = {};
    m_stumbleStepsTaken = 0;
    m_supportLost = {};
    m_swingTrajectory = {};
    m_sinceRecoveryLanding = 10.0f;
    m_initialized = true;
}

Vec3 ContactFootwork3D::wantedLanding(const ContactFootworkInput3D& input,
    std::size_t side, Quaternion& orientation) const {
    const FootPlan3D& foot = m_plan.feet[side];
    const FootReference3D& reference = input.reference[side];
    orientation = reference.poseOrientationWorld;
    if (foot.reason == StepReason3D::Settle && input.standing && input.assistHold > 0.5f)
        return pairStanceTarget(input, side);
    if (foot.reason != StepReason3D::Turn || !input.standing) return reference.poseWorld;
    // Giro: a pose parada ja no rumo final da pelve, ate o limite por par a
    // partir do rumo em que o passo comecou - no sentido em que o passo
    // comecou. Girando sem parar, o alvo passava de 180 graus do inicio do
    // passo, o sentido "mais curto" virava e o pouso saltava 60 cm para o
    // outro lado (o pe a 9 m/s).
    float remaining = wrapAngle(input.standingTurnTargetYawRadians - foot.turnFromFacing);
    if (m_turnSign[side] > 0.0f && remaining < -0.5f * Pi) remaining += 2.0f * Pi;
    if (m_turnSign[side] < 0.0f && remaining > 0.5f * Pi) remaining -= 2.0f * Pi;
    const float finalYaw = foot.turnFromFacing + std::clamp(remaining, -TurnStepLimit, TurnStepLimit);
    const Quaternion spin = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
        wrapAngle(finalYaw - input.facingYawRadians));
    orientation = (spin * reference.poseOrientationWorld).normalized();
    return input.referenceRootWorld + spin.rotate(reference.poseWorld - input.referenceRootWorld);
}

Vec3 ContactFootwork3D::pairStanceTarget(const ContactFootworkInput3D& input, std::size_t side) const {
    // Onde a pose parada quer este pe em relacao ao outro (apoiado).
    const std::size_t other = 1 - side;
    Vec3 target = m_plan.feet[other].anchorWorld
        + (input.reference[side].poseWorld - input.reference[other].poseWorld);
    target.z = input.reference[side].poseWorld.z;
    return target;
}

float ContactFootwork3D::stanceYaw(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input) const {
    // Rumo da base: o de cada pe apoiado menos o quanto a pose parada abre
    // aquele pe em relacao a pelve.
    float sine = 0.0f, cosine = 0.0f;
    for (std::size_t side = 0; side < 2; ++side) {
        const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
        const float open = wrapAngle(footYaw3D(link, input.reference[side].poseOrientationWorld)
            - input.facingYawRadians);
        const float yaw = footYaw3D(link, m_plan.feet[side].anchorOrientationWorld) - open;
        sine += std::sin(yaw);
        cosine += std::cos(yaw);
    }
    return std::atan2(sine, cosine);
}

void ContactFootwork3D::startStep(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, StepReason3D reason,
    SwingPath3D path, float urgency) {
    FootPlan3D& foot = m_plan.feet[side];
    // Comeco de um giro (o outro pe nao esta girando nem espera o segundo
    // passo): o centro em volta do qual a base gira.
    if (reason == StepReason3D::Turn && m_pendingTurnFoot < 0
        && m_plan.feet[1 - side].reason != StepReason3D::Turn) {
        m_turnCenter = (m_plan.feet[0].anchorWorld + m_plan.feet[1].anchorWorld) * 0.5f;
        m_turnCenterValid = true;
    }
    foot.phase = FootPhase3D::Unloading;
    foot.reason = reason;
    foot.path = path;
    foot.stepId = m_nextStepId++;
    foot.revision = 0;
    foot.phaseSeconds = 0.0f;
    foot.swingSeconds = 0.0f;
    foot.urgency = urgency;
    foot.turnFromFacing = input.facingYawRadians;
    m_turnSign[side] = wrapAngle(input.standingTurnTargetYawRadians - input.facingYawRadians) >= 0.0f
        ? 1.0f : -1.0f;
    foot.extraClearance = 0.0f;
    foot.touchdownEarly = false;
    foot.touchdownEvidence = ContactEvidence3D::None;
    foot.releasedUnloaded = false;
    // Ate o pe soltar, o alvo do passo e de onde ele sai.
    m_recoveryTarget[side] = foot.anchorWorld;
    foot.goalWorld = foot.anchorWorld;
    foot.goalOrientationWorld = foot.anchorOrientationWorld;
    if (reason == StepReason3D::Recovery) {
        foot.swingDurationSeconds = m_settings.recoveryCaptureTargeting
            ? 0.20f : recoveryStepSeconds(urgency);
    } else {
        Quaternion orientation;
        const Vec3 position = wantedLanding(input, side, orientation);
        const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
        const float distance = std::hypot(position.x - foot.anchorWorld.x,
            position.y - foot.anchorWorld.y);
        const float yaw = std::abs(wrapAngle(footYaw3D(link, orientation)
            - footYaw3D(link, foot.anchorOrientationWorld)));
        foot.swingDurationSeconds = reason == StepReason3D::Turn
            ? turnStepSeconds(distance, yaw) : ordinaryStepSeconds(distance, yaw);
        // Ajuda cheia: o passo de giro na velocidade do giro pedido.
        if (reason == StepReason3D::Turn && input.assistHold > 0.5f) {
            const float need = std::clamp(std::abs(input.standingTurnRateRadiansPerSecond) / 6.0f
                + std::abs(wrapAngle(input.standingTurnTargetYawRadians - input.facingYawRadians)) / 1.5f, 0.0f, 1.0f);
            foot.swingDurationSeconds = HeldTurnSwingSlow + (HeldTurnSwingFast - HeldTurnSwingSlow) * need;
        }
    }
    ++m_plan.stepsStarted;
}

void ContactFootwork3D::makeRecovery(const ContactFootworkInput3D& input, std::size_t side,
    float urgency) {
    // Um passo de acomodacao ou de giro em andamento vira de recuperacao no
    // ritmo dela, sem pular no caminho: o progresso e a forma do caminho
    // ficam; muda a razao, o ritmo e o alvo (a pose sob o corpo).
    FootPlan3D& foot = m_plan.feet[side];
    const float duration = recoveryStepSeconds(urgency);
    if (foot.swingDurationSeconds > duration) {
        const float progress = foot.swingSeconds / std::max(0.01f, foot.swingDurationSeconds);
        foot.swingDurationSeconds = duration;
        foot.swingSeconds = progress * duration;
    }
    if (foot.reason != StepReason3D::Recovery) {
        m_recoveryTarget[side] = foot.goalWorld;
        foot.reason = StepReason3D::Recovery;
        ++foot.revision;
        ++m_plan.reasonChanges;
        // Sem a ajuda (captura): o passo que ja ia no ar recomeca de onde o
        // pe esta, com o tempo de um passo de recuperacao e mirando a
        // captura. Mantido o progresso, um passo de acomodacao ja adiantado
        // (alvo so muda antes de 60%) pousava onde ia, perto de onde saiu, e
        // o mesmo pe - agora carregado - tinha de sair de novo (arrastado).
        if (m_settings.recoveryCaptureTargeting && airborne(foot.phase)) {
            const PhysicsBodyState3D& body = input.state->links[m_feet[side]];
            foot.liftoffWorld = body.position;
            foot.liftoffOrientationWorld = body.orientation;
            foot.phase = FootPhase3D::Swing;
            foot.swingSeconds = 0.0f;
            foot.swingDurationSeconds = 0.20f;
            m_recoveryTarget[side] = body.position;
        }
    }
    foot.urgency = std::max(foot.urgency, urgency);
}

void ContactFootwork3D::release(const RagdollProfile3D&,
    const ContactFootworkInput3D& input, std::size_t side, ReleaseReason3D reason) {
    // Sai de onde o pe esta de fato (medido), nao da pose ideal.
    FootPlan3D& foot = m_plan.feet[side];
    foot.releaseReason = reason;
    foot.physicalRelease = PhysicalRelease3D::Releasing;
    ++m_plan.releasesByReason[static_cast<std::size_t>(reason)];
    const PhysicsBodyState3D& body = input.state->links[m_feet[side]];
    foot.releasedUnloaded = m_plan.loadShare[side] < UnloadedShare;
    if (!foot.releasedUnloaded) {
        ++m_plan.forcedReleases;
        if (foot.reason == StepReason3D::Locomotion) ++m_plan.locomotionForcedReleases;
    }
    foot.liftoffWorld = body.position;
    foot.liftoffOrientationWorld = body.orientation;
    foot.liftoffGroundHeight = foot.surface.height;
    foot.phase = FootPhase3D::Swing;
    foot.phaseSeconds = 0.0f;
    foot.swingSeconds = 0.0f;
    m_releaseShiftFraction[side] = 0.0f;
    m_releaseShiftLimit[side] = 0.0f;
    m_ticksWithoutContact[side] = 0;
    m_liftedOff[side] = false;
    m_fitValid[side] = false;
    m_swingTrajectory[side] = {};
}

void ContactFootwork3D::seedFoot(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side) {
    // Rente ao chao: apoio onde esta (nunca puxar uma perna erguida para
    // baixo nem mover o corpo simulado na troca). Erguido: um passo dali ate
    // a base, que so pousa por contato.
    FootPlan3D& foot = m_plan.feet[side];
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const PhysicsBodyState3D& body = input.state->links[m_feet[side]];
    const FootSurface3D ground = input.surfaceAt
        ? input.surfaceAt(side, body.position, footForwardWorld3D(link, body.orientation))
        : FootSurface3D {};
    const float height = footLockHeight3D(link, ground.height, body.orientation);
    foot.followingGait = false;
    foot.anchorWorld = body.position;
    foot.anchorOrientationWorld = body.orientation;
    foot.surface = ground;
    // Apoiado de fato (contato medido), ou rente e parado: apoio onde esta.
    // No ar e perto do chao (o fim de uma passada, a parada): desce onde ja
    // ia pousar, rapido - ir ate a pose num passo inteiro deixava os dois pes
    // no ar ao mesmo tempo na parada, e os bracos chicoteavam. Alto (a perna
    // erguida do fim do levantar): um passo ate a base. Os dois so pousam
    // por contato; a acomodacao arruma a base depois.
    const FootSupportEstimate3D& measured = input.physical->feet[side];
    const Vec3 soleVelocity = measured.soleVelocityWorld;
    // (Em contato mas escorregando rapido - a freada de uma corrida - nao e
    // apoio ainda: travado ali, a perna o arrastava de volta ate a trava.)
    const float soleSpeed = std::hypot(soleVelocity.x, soleVelocity.y);
    const bool resting = (contactSupport(measured) && soleSpeed <= 0.50f)
        || (ground.valid && std::abs(body.position.z - height) <= 0.08f && soleSpeed <= 0.30f);
    if (!resting && ground.valid && body.position.z - height <= 0.15f) {
        startStep(profile, input, side, StepReason3D::Landing, SwingPath3D::Direct, 0.0f);
        const Vec3 target = body.position + clampMagnitude(
            Vec3 { soleVelocity.x, soleVelocity.y, 0.0f } * 0.08f, 0.10f);
        // Rapido perto do chao, mais devagar de mais alto (nascer 15 cm acima
        // do chao: o corpo desce junto, sem despencar).
        foot.swingDurationSeconds = std::clamp(0.10f + 1.2f * (body.position.z - height)
            + 0.5f * std::hypot(target.x - body.position.x, target.y - body.position.y), 0.12f, 0.34f);
        m_recoveryTarget[side] = target;
        release(profile, input, side, ReleaseReason3D::Recovery);
        return;
    }
    if (!resting) {
        startStep(profile, input, side, StepReason3D::ReestablishSupport,
            SwingPath3D::Direct, 0.0f);
        foot.swingDurationSeconds = 0.34f;
        release(profile, input, side, ReleaseReason3D::Recovery);
        return;
    }
    foot.anchorWorld.z = height;
    foot.phase = FootPhase3D::Stance;
    foot.reason = StepReason3D::None;
    foot.phaseSeconds = 0.0f;
    foot.targetWorld = foot.anchorWorld;
    foot.targetOrientationWorld = foot.anchorOrientationWorld;
}

void ContactFootwork3D::land(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side) {
    // Tocou: o apoio e onde o pe tocou (contato fora do alvo pede replano do
    // corpo, nao arrastar a sola ate o alvo), com a sola plana no chao dali.
    FootPlan3D& foot = m_plan.feet[side];
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const PhysicsBodyState3D& body = input.state->links[m_feet[side]];
    // A altura e a da sola medida (ela esta tocando): no plano ela repousa
    // alguns milimetros acima do chao da sonda (a margem do contato), e
    // ancorar pela sonda apertava o pe contra o chao - a perna ficava dura
    // (varredura de empurroes: 12 quedas contra 3). A sonda so vale quando a
    // sola esta bem acima dela (mais de 1,5 cm: um pe inclinado tocando a
    // quina de um degrau); ela nunca vale acima da sola (ve o degrau vizinho
    // pela ponta do pe: 8 cm acima de onde o pe estava).
    Quaternion orientation = foot.goalOrientationWorld;
    FootSurface3D surface = input.surfaceAt
        ? input.surfaceAt(side, body.position, footForwardWorld3D(link, orientation))
        : FootSurface3D {};
    // (Pe arrastado - nunca saiu do chao, deslizou ate aqui, talvez pela
    // quina de um degrau: a sonda, se estiver abaixo da sola.)
    const float sole = input.physical->feet[side].soleLowestHeight;
    surface.height = !m_liftedOff[side] && surface.valid ? std::min(surface.height, sole) : sole;
    surface.valid = true;
    Vec3 anchor = body.position;
    anchor.z = footLockHeight3D(link, surface.height, orientation);
    foot.touchdownEarly = foot.phase == FootPhase3D::Swing
        && foot.swingSeconds < foot.swingDurationSeconds;
    foot.touchdownEvidence = input.physical->feet[side].evidence;
    if (foot.reason == StepReason3D::Recovery) {
        m_lastRecoverySide = static_cast<int>(side);
        m_sinceRecoveryLanding = 0.0f;
    }
    if (foot.reason == StepReason3D::Locomotion) {
        // O intervalo real entre pousos (descarga, balanco, busca, dupla
        // sustentacao), medido por eventos.
        if (m_lastLocomotionSide >= 0 && m_sinceLocomotionTouchdown < 1.5f)
            m_locomotionPeriod += (m_sinceLocomotionTouchdown - m_locomotionPeriod) * 0.5f;
        m_lastLocomotionSide = static_cast<int>(side);
        m_sinceLocomotionTouchdown = 0.0f;
    }
    if (foot.touchdownEarly) ++m_plan.earlyTouchdowns;
    ++m_plan.touchdownsByContact;
    // Nunca saiu do chao: foi arrastado ate aqui (a carga nao passou). O
    // proximo passo comum deste pe espera, cada vez mais; um passo de
    // verdade zera a espera.
    foot.dragged = !m_liftedOff[side] && foot.path != SwingPath3D::Pivot;
    if (foot.dragged) {
        ++m_plan.draggedSteps;
        m_draggedBackoff[side] = std::min(DraggedCooldownMaximumSeconds,
            m_draggedBackoff[side] > 0.0f ? m_draggedBackoff[side] * 2.0f : DraggedCooldownSeconds);
        if (foot.reason != StepReason3D::Recovery) m_stepCooldown[side] = m_draggedBackoff[side];
    } else {
        m_draggedBackoff[side] = 0.0f;
    }
    foot.anchorWorld = anchor;
    foot.anchorOrientationWorld = orientation;
    foot.surface = surface;
    foot.physicalRelease = PhysicalRelease3D::Grounded;
    foot.phase = FootPhase3D::Loading;
    foot.phaseSeconds = 0.0f;
    foot.targetWorld = anchor;
    foot.targetOrientationWorld = orientation;
}

Vec3 ContactFootwork3D::locomotionLanding(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, Vec3 pose,
    Quaternion orientation) const {
    // Onde pousar para o corpo MANTER a velocidade pedida (nao parar): pelo
    // pendulo invertido sobre o pe de apoio, o ponto de captura no pouso
    // (xi(T) = p + e^(wT) (xi0 - p)) menos o afastamento de uma passada
    // periodica, delta = v T / (e^(wT) - 1) - com o pe ali, no fim do proximo
    // passo o corpo estara na mesma velocidade. De lado, cada pe do seu lado.
    const FootPlan3D& foot = m_plan.feet[side];
    const CharacterPhysicalState3D& physical = *input.physical;
    const std::size_t other = 1 - side;
    const Vec3 support = physical.feet[other].supporting ? physical.feet[other].soleCenterWorld
        : m_plan.feet[other].anchorWorld;
    const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
    const float remaining = std::max(0.05f, foot.swingDurationSeconds - foot.swingSeconds);
    Vec3 capture = input.capturePointWorld;
    capture = support + (capture - support) * std::exp(omega * remaining);
    // A velocidade no fim do proximo passo: a pedida, ate o que um passo
    // muda (arrancar e frear em passos, ~1,2 m/s por passo).
    Vec3 current = input.planningComVelocityWorld;
    current.z = 0.0f;
    Vec3 desired = input.desiredVelocityWorld;
    desired.z = 0.0f;
    desired = current + clampMagnitude(desired - current, LocomotionSpeedChangePerStep);
    // Passadas alternadas: com o pe de apoio em p_k, o que sai pousando em
    // p_k+1 e o seguinte em p_k+2, os deslocamentos sao d + s 2w e d - s 2w
    // (d = v T, 2w a largura da base, s o lado de quem sai). A orbita
    // periodica pede no pouso ponto de captura - p_k+1 = d/(E-1) - s 2w/(E+1),
    // E = e^(wT) (guia 17, recorrencia de dois apoios): o termo lateral
    // alterna de sinal - somar meia largura depois da translacao nao fecha o
    // ciclo. T e o intervalo medido entre pousos.
    const float period = std::clamp(m_locomotionPeriod, 0.25f, 0.80f);
    const float growth = std::exp(omega * period);
    const Vec3 offset = desired * (period / std::max(0.05f, growth - 1.0f));
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const Vec3 half = ragdollColliderHasBox3D(link.collider)
        ? link.collider.boxHalfExtents : Vec3 {};
    Vec3 soleOffset = orientation.rotate(link.collider.localPosition
        + link.collider.localOrientation.rotate({ 0.0f, 0.0f, -half.z }));
    soleOffset.z = 0.0f;
    // Cada pe a meia largura da base ao lado da trajetoria do ponto de
    // captura (de lado, o de tras fecha perto do da frente: com um minimo de
    // 14 cm do pe de apoio ele ficava longe e o corpo passava dos dois); nunca
    // cruzando (8 cm).
    const Vec3 outward = input.heading.rotate({ 0.0f, side == 0 ? 1.0f : -1.0f, 0.0f });
    // A largura cresce com o deslocamento de lado: entre apoios alternados os
    // passos sao d + 2w e d - 2w, e o pe que fecha so nao cruza o outro com
    // 2w >= |d de lado| + a largura minima. Com 2w fixo em 20 cm, andando de
    // lado a ~0,7 m/s (d ~0,35 m) o plano pedia o pe de tras cruzando na
    // frente do outro; a regra anti-cruzamento travava, ele pousava 37 cm
    // atras e o corpo passava dos dois pes.
    const float sidewaysStep = std::abs(dot(desired * period, outward));
    const float width = LocomotionStepWidth + sidewaysStep;
    Vec3 target = capture - offset - soleOffset + outward * (width / (growth + 1.0f));
    const float lateral = dot(target - support, outward);
    if (lateral < 0.08f) target += outward * (0.08f - lateral);
    Vec3 fromSupport = target - support;
    fromSupport.z = 0.0f;
    if (fromSupport.length() > 0.80f)
        target = support + fromSupport * (0.80f / fromSupport.length());
    target.z = pose.z;
    return target;
}

Vec3 ContactFootwork3D::predictedLocomotionLanding(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, Vec3 pose, Quaternion orientation,
    const Vec3* committed) const {
    // Fase 11 (pacote do GPT): o pouso nominal (acima) e o centro de alguns
    // candidatos; cada um e avaliado pelo que o corpo faz ate o pouso depois
    // do proximo, na agenda da marcha - apoio no pe atual ate o toque, dupla
    // sustentacao, apoio no candidato pelo balanco do outro, e o passo
    // seguinte pousando pelo nominal dentro do alcance e sem cruzar - com o
    // corpo pela tarefa corporal (supportAdvance). Custo: a velocidade media
    // contra a pedida, o que os dois passos seguintes nao alcancam (o resto
    // da projecao: de lado, o pe que fecha nao passa o da frente, e so este
    // alcanca o ponto de captura) e a mudanca do alvo ja comprometido.
    const Vec3 nominalTarget = locomotionLanding(profile, input, side, pose, orientation);
    const FootPlan3D& foot = m_plan.feet[side];
    const CharacterPhysicalState3D& physical = *input.physical;
    const std::size_t other = 1 - side;
    const auto planar = [](Vec3 v) { return Vec3 { v.x, v.y, 0.0f }; };
    const Vec3 support = planar(physical.feet[other].supporting ? physical.feet[other].soleCenterWorld
        : m_plan.feet[other].anchorWorld);
    const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
    const float remaining = std::max(0.0f, foot.swingDurationSeconds - foot.swingSeconds);
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const Vec3 half = ragdollColliderHasBox3D(link.collider)
        ? link.collider.boxHalfExtents : Vec3 {};
    const Vec3 soleOffset = planar(orientation.rotate(link.collider.localPosition
        + link.collider.localOrientation.rotate({ 0.0f, 0.0f, -half.z })));
    Vec3 current = planar(input.planningComVelocityWorld);
    Vec3 desired = planar(input.desiredVelocityWorld);
    desired = current + clampMagnitude(desired - current, LocomotionSpeedChangePerStep);
    const float desiredSpeed = desired.length();
    const float period = std::clamp(m_locomotionPeriod, 0.25f, 0.80f);
    const float growth = std::exp(omega * period);
    const Vec3 offset = desired * (period / std::max(0.05f, growth - 1.0f));
    const float swing = locomotionSwingSeconds(desiredSpeed);
    const auto outwardOf = [&](std::size_t foot) {
        return input.heading.rotate({ 0.0f, foot == 0 ? 1.0f : -1.0f, 0.0f });
    };
    // O nominal de um pe (sola) com o ponto de captura no pouso e o apoio
    // do outro, projetado no alcance e sem cruzar; devolve o resto.
    const auto place = [&](Vec3 capture, Vec3 from, std::size_t foot, float& residual) {
        const Vec3 outward = outwardOf(foot);
        const float width = LocomotionStepWidth + std::abs(dot(desired * period, outward));
        const Vec3 ideal = capture - offset + outward * (width / (growth + 1.0f));
        Vec3 placed = ideal;
        const float lateral = dot(placed - from, outward);
        if (lateral < 0.08f) placed += outward * (0.08f - lateral);
        const Vec3 reach = placed - from;
        if (reach.length() > 0.80f) placed = from + reach * (0.80f / reach.length());
        residual = (ideal - placed).length();
        return placed;
    };
    const PlanarBody3D start { planar(physical.centerOfMassWorld), current };
    const PlanarBody3D touchdown = supportAdvance(start, support, support, desired, omega, remaining);
    const float horizon = remaining + 2.0f * (PredictionDoubleSupportSeconds + swing);
    const Vec3 nominal = planar(nominalTarget) + soleOffset;
    const Vec3 along = desiredSpeed > 0.1f ? desired * (1.0f / desiredSpeed)
        : input.heading.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 across { -along.y, along.x, 0.0f };
    const Vec3 committedSole = committed ? planar(*committed) + soleOffset : nominal;
    float bestCost = std::numeric_limits<float>::infinity();
    Vec3 best = nominal;
    for (const float forward : { -0.10f, -0.05f, 0.0f, 0.05f, 0.10f, 0.15f, 0.20f }) {
        for (const float sideways : { -0.05f, 0.0f, 0.05f }) {
            Vec3 candidate = nominal + along * forward + across * sideways;
            // O proprio candidato: sem cruzar e ao alcance do apoio.
            const float lateral = dot(candidate - support, outwardOf(side));
            if (lateral < 0.08f) candidate += outwardOf(side) * (0.08f - lateral);
            const Vec3 reach = candidate - support;
            if (reach.length() > 0.80f) candidate = support + reach * (0.80f / reach.length());
            // Descarga do outro pe, e ele pousa pelo nominal.
            PlanarBody3D body = supportAdvance(touchdown, support, candidate, desired, omega,
                PredictionDoubleSupportSeconds);
            body = supportAdvance(body, candidate, candidate, desired, omega, swing);
            float residualNext = 0.0f, residualAfter = 0.0f;
            const Vec3 next = place(body.com + body.velocity * (1.0f / omega), candidate, other, residualNext);
            // Descarga deste pe, e ele pousa de novo.
            body = supportAdvance(body, candidate, next, desired, omega, PredictionDoubleSupportSeconds);
            body = supportAdvance(body, next, next, desired, omega, swing);
            place(body.com + body.velocity * (1.0f / omega), next, side, residualAfter);
            const Vec3 mean = (body.com - start.com) * (1.0f / std::max(0.1f, horizon));
            const float cost = (mean - desired).lengthSquared()
                + 10.0f * (residualNext * residualNext + residualAfter * residualAfter)
                + 2.0f * (candidate - committedSole).lengthSquared();
            if (cost < bestCost) {
                bestCost = cost;
                best = candidate;
            }
        }
    }
    Vec3 target = best - soleOffset;
    target.z = pose.z;
    return target;
}

Vec3 ContactFootwork3D::captureLanding(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, Vec3 pose,
    Quaternion orientation) const {
    // Onde o ponto de captura vai estar quando o pe pousar (pendulo invertido
    // sobre o pe de apoio: xi(T) = p + e^(wT) (xi0 - p)): la o pe para o
    // corpo. Mirar a pose sob o corpo so bastava com a pelve empurrada de
    // volta pela ajuda; sem ela o corpo passava do pe e caia.
    const FootPlan3D& foot = m_plan.feet[side];
    const CharacterPhysicalState3D& physical = *input.physical;
    const std::size_t other = 1 - side;
    Vec3 support = physical.feet[other].supporting ? physical.feet[other].soleCenterWorld
        : m_plan.feet[other].anchorWorld;
    const float remaining = std::max(0.05f, foot.swingDurationSeconds - foot.swingSeconds);
    const float growth = std::exp(std::clamp(physical.captureOmega, 1.0f, 6.0f) * remaining);
    Vec3 capture = input.capturePointWorld;
    capture = support + (capture - support) * growth;
    capture.z = pose.z;
    // Da sola (onde vai o ponto de captura) para a origem do link (o que o IK
    // recebe), na orientacao do pouso.
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const Vec3 half = ragdollColliderHasBox3D(link.collider)
        ? link.collider.boxHalfExtents : Vec3 {};
    Vec3 soleOffset = orientation.rotate(link.collider.localPosition
        + link.collider.localOrientation.rotate({ 0.0f, 0.0f, -half.z }));
    soleOffset.z = 0.0f;
    Vec3 target = capture - soleOffset;
    // Sem cruzar as pernas: o pe fica do proprio lado do de apoio (12 cm).
    const Vec3 outward = input.heading.rotate({ 0.0f, side == 0 ? 1.0f : -1.0f, 0.0f });
    const float lateral = dot(target - support, outward);
    if (lateral < 0.12f) target += outward * (0.12f - lateral);
    // Alcance: ate 75 cm do pe de apoio; alem disso, o maior passo que cabe
    // (o proximo passo continua).
    Vec3 fromSupport = target - support;
    fromSupport.z = 0.0f;
    if (fromSupport.length() > 0.75f)
        target = support + fromSupport * (0.75f / fromSupport.length()) + Vec3 { 0.0f, 0.0f, target.z - support.z };
    target.z = pose.z;
    return target;
}

Vec3 ContactFootwork3D::brakeLanding(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, Vec3 pose,
    Quaternion orientation) const {
    // Passo de freada, pelo pendulo invertido sobre o pe de apoio real: onde
    // o centro de massa e o ponto de captura estarao no toque. Rapido, o pe
    // pousa a frente do centro de massa a uma distancia em que a forca do
    // chao que freia passa por ele (d = h a / g): mais longe o chao gira o
    // corpo para tras (com o pe a 1 m ele caia de costas), mais perto o pe
    // nao freia. Mas nunca alem do ponto de captura: devagar (ou com o pe da
    // frente ja freando), o pouso e o ponto de captura - o corpo para em
    // cima do pe. Mirando sempre a distancia de freada, os ultimos passos
    // pousavam a frente de um corpo que ja parava, e ele caia de costas com
    // os dois pes a frente.
    const FootPlan3D& foot = m_plan.feet[side];
    const CharacterPhysicalState3D& physical = *input.physical;
    const std::size_t other = 1 - side;
    // O outro pe: apoiado, a sola; no plano em apoio, a ancora; no ar, onde
    // ele esta de fato (a ancora dele e a do ultimo apoio, as vezes metros
    // atras - o alcance medido dali puxava o alvo 1,5 m para tras).
    const bool otherDown = physical.feet[other].supporting || supportsWeight(m_plan.feet[other].phase);
    const Vec3 support = physical.feet[other].supporting || !otherDown
        ? physical.feet[other].soleCenterWorld : m_plan.feet[other].anchorWorld;
    Vec3 direction = m_stumbleStep[side] ? input.stumbleDirectionWorld : input.brakeDirectionWorld;
    direction.z = 0.0f;
    if (direction.lengthSquared() < 1e-4f) direction = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
    direction = direction.normalized();
    const float remaining = std::max(0.03f, foot.swingDurationSeconds - foot.swingSeconds);
    const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
    Vec3 velocity = input.planningComVelocityWorld;
    velocity.z = 0.0f;
    // Pendulo sobre o apoio: x(T) = p + (x0 - p) ch + v0/w sh; xi(T) = p +
    // (xi0 - p) e^(wT). Sem o outro pe no chao (os dois no ar, a troca da
    // corrida para os passos), o corpo voa ate o toque: balistico. (Pelo
    // pendulo em volta da ancora velha do pe no ar, metros atras, o alvo
    // saia 3 m atras do corpo.)
    const Vec3 p { support.x, support.y, 0.0f };
    const Vec3 x0 { physical.centerOfMassWorld.x, physical.centerOfMassWorld.y, 0.0f };
    Vec3 capture = input.capturePointWorld;
    capture.z = 0.0f;
    Vec3 comTouchdown, captureTouchdown;
    if (otherDown) {
        // Com o pe de apoio freando, pela aceleracao medida do corpo: o
        // pendulo ideal sobre o pe nao ve a perna cedendo, o atrito e o
        // equilibrio empurrando - previa o corpo passando do pe (captura a
        // +0,8 m) quando ele parava antes, o passo mirava longe e vinham
        // passos de correcao para tras.
        Vec3 acceleration = physical.centerOfMassAccelerationWorld;
        acceleration.z = 0.0f;
        const float alongAcceleration = std::clamp(dot(acceleration, direction), -12.0f, 3.0f);
        const Vec3 a = direction * alongAcceleration;
        const float alongSpeed = dot(velocity, direction);
        // Ate parar no rumo (a aceleracao medida nao segue depois disso).
        const float t = alongAcceleration < -0.01f
            ? std::min(remaining, std::max(0.0f, -alongSpeed / alongAcceleration)) : remaining;
        comTouchdown = x0 + velocity * t + a * (0.5f * t * t);
        const Vec3 touchdownVelocity = velocity + a * t;
        captureTouchdown = comTouchdown + touchdownVelocity * (1.0f / omega);
    } else if (otherDown) {
        const float c = std::cosh(omega * remaining), sh = std::sinh(omega * remaining);
        comTouchdown = p + (x0 - p) * c + velocity * (sh / omega);
        captureTouchdown = p + (capture - p) * std::exp(omega * remaining);
    } else {
        comTouchdown = x0 + velocity * remaining;
        captureTouchdown = comTouchdown + velocity * (1.0f / omega);
    }
    // O alcance do pe de freada depende do rumo em relacao ao corpo: a
    // frente o quadril flexiona muito; de lado (abducao) e para tras
    // (extensao, ~30 graus) bem menos - para tras, mirando 0,55 m, o pe
    // subia ate 0,76 m procurando um alvo fora do alcance e o corpo tombava.
    const Vec3 bodyForward = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 bodyLeft = input.heading.rotate({ 0.0f, 1.0f, 0.0f });
    const float forwardness = dot(direction, bodyForward);
    const float sideways = dot(direction, bodyLeft);
    const float sideReach = BrakePlaceSideways;
    const float placeMax = sideReach + forwardness * forwardness
        * ((forwardness >= 0.0f ? BrakePlaceForward : BrakePlaceBackward)
            - sideReach);
    const float height = std::max(0.4f, physical.centerOfMassWorld.z - support.z);
    const float braking = std::clamp(height * BrakePlaceDeceleration / 9.81f,
        BrakePlaceMinimum, placeMax);
    const float comAlong = dot(comTouchdown, direction);
    // Um pouco atras do ponto de captura: o corpo para entre os pes, nao na
    // borda de tras da base (parado ali, com os dois pes a frente, o pendulo
    // caia para tras).
    // (De lado mais: o pe de fora segue freando depois do toque do de
    // dentro e o corpo parava atras dos dois.)
    const float captureAlong = dot(captureTouchdown, direction)
        - (BrakeStraddle + (BrakeStraddleSideways - BrakeStraddle) * std::abs(sideways));
    // De lado, o pe de dentro so fecha a base: vai a uma largura de quadril
    // do pe de fora, sem cruzar (a passada de quem freia de lado).
    // (Nem ele alem do ponto de captura: fechando a frente do corpo que ja
    // parava, a base ficava toda adiante e o corpo escorregava de volta.)
    float along = m_brakeClosing[side]
        ? std::min(dot(p, direction) - BrakeHipWidth, captureAlong)
        : std::min(comAlong + braking, captureAlong);
    // Quase-queda: um pouco a frente de onde o corpo vai estar no toque - o
    // pe pega o corpo que cai para a frente sem freia-lo (ele segue correndo).
    if (m_stumbleStep[side]) along = comAlong + input.stumbleAheadMeters;
    if (m_brakeRearLanding[side]) {
        // Apoio de tras do afundo: onde o pe ja vai, nunca a frente do corpo.
        const Vec3 now = physical.feet[side].soleCenterWorld;
        along = std::min(dot(Vec3 { now.x, now.y, 0.0f }, direction) + 0.05f,
            comAlong - BrakeRearBehind);
    }
    // Atravessado ao rumo: o ponto de captura; cada pe do seu lado do corpo
    // (andando para a frente ou para tras a base e larga; de lado, o rumo ja
    // separa os pes).
    const Vec3 left { -direction.y, direction.x, 0.0f };
    const Vec3 outwardBody = bodyLeft * (side == 0 ? 1.0f : -1.0f);
    Vec3 sole = direction * along + left * dot(captureTouchdown, left)
        + outwardBody * (BrakeHalfWidth * (1.0f - std::abs(sideways)));
    // Da sola para a origem do link (o que o IK recebe), na orientacao do pouso.
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const Vec3 half = ragdollColliderHasBox3D(link.collider)
        ? link.collider.boxHalfExtents : Vec3 {};
    Vec3 soleOffset = orientation.rotate(link.collider.localPosition
        + link.collider.localOrientation.rotate({ 0.0f, 0.0f, -half.z }));
    soleOffset.z = 0.0f;
    Vec3 target = sole - soleOffset;
    // Sem cruzar as pernas (no referencial do corpo), e ao alcance a partir
    // do pe de apoio.
    const float lateral = dot(target - p, outwardBody);
    if (lateral < 0.12f) target += outwardBody * (0.12f - lateral);
    Vec3 fromSupport = target - p;
    fromSupport.z = 0.0f;
    const float reach = BrakeReach;
    // (O passo de quase-queda e medido do corpo, nao do outro pe: os dois
    // podem estar atras dele.)
    if (fromSupport.length() > reach && !m_stumbleStep[side])
        target = p + fromSupport * (reach / fromSupport.length());
    target.z = pose.z;
    return target;
}

void ContactFootwork3D::executeSwing(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& input, std::size_t side, float deltaTime) {
    FootPlan3D& foot = m_plan.feet[side];
    const FootPlan3D& support = m_plan.feet[1 - side];
    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
    const FootSupportEstimate3D& measured = input.physical->feet[side];
    foot.phaseSeconds += deltaTime;
    foot.swingSeconds += deltaTime;
    if (foot.physicalRelease == PhysicalRelease3D::Releasing && !measured.contactObserved)
        foot.physicalRelease = PhysicalRelease3D::Airborne;
    const float duration = std::max(0.05f, foot.swingDurationSeconds);
    const float progress = std::clamp(foot.swingSeconds / duration, 0.0f, 1.0f);
    const float eased = smoothStep(progress);
    const float arch = std::sin(Pi * progress);

    // Para onde o pe vai AGORA (o corpo continua se mexendo durante o passo,
    // e a camera pode continuar girando). A recuperacao acompanha o corpo
    // ate ~60% do passo; depois o pe so termina de pousar.
    Quaternion plannedOrientation;
    Vec3 plannedPosition = wantedLanding(input, side, plannedOrientation);
    if (foot.reason == StepReason3D::Recovery) {
        if (progress < 0.6f) {
            if (m_settings.recoveryCaptureTargeting) {
                // Corpo indo depressa (empurrao, arrasto): o pe vai ao ponto
                // de captura. So balancando parado (0,1-0,3 m/s): volta para
                // a base (a pose) - mirando a captura ali, a base perdia a
                // forma e ele nao parava de se acomodar (varredura de parado:
                // 105 passos em 15 nascimentos, nenhum quieto).
                const Vec3 capture = captureLanding(profile, input, side, plannedPosition, plannedOrientation);
                const float speed = std::hypot(input.planningComVelocityWorld.x,
                    input.planningComVelocityWorld.y);
                const float toCapture = smoothStep((speed - CaptureTargetSlowSpeed)
                    / (CaptureTargetFastSpeed - CaptureTargetSlowSpeed));
                m_recoveryTarget[side] = plannedPosition + (capture - plannedPosition) * toCapture;
            } else {
                m_recoveryTarget[side] = plannedPosition;
            }
        }
        plannedPosition = m_recoveryTarget[side];
    } else if (foot.reason == StepReason3D::Locomotion && m_settings.locomotionStepping) {
        // Andando pelo planejador: onde o corpo mantem a velocidade pedida.
        // O alvo comprometido acompanha o pedido (ver SwingFreeRevision...);
        // o pe vai a ele pela trajetoria continua (fase 10: interpolado da
        // soltura ao alvo do tick, o alvo saltava ate 10 cm num tick na
        // parada ou de lado e o pe perseguia o salto).
        SwingTrajectory3D& path = m_swingTrajectory[side];
        const bool pathOfThisStep = path.valid && path.stepId == foot.stepId;
        if (m_settings.locomotionPlacementPredictor) ++m_plan.placementPredictorCalls;
        const Vec3 wanted = m_settings.locomotionPlacementPredictor
            ? predictedLocomotionLanding(profile, input, side, plannedPosition, plannedOrientation,
                pathOfThisStep ? &path.committed : nullptr)
            : locomotionLanding(profile, input, side, plannedPosition, plannedOrientation);
        if (!path.valid || path.stepId != foot.stepId) {
            path = {};
            path.valid = true;
            path.stepId = foot.stepId;
            path.position = { foot.liftoffWorld.x, foot.liftoffWorld.y, 0.0f };
            path.committed = wanted;
            const float swing = std::max(0.05f, foot.swingDurationSeconds);
            path.acceleration = Vec3 { wanted.x - foot.liftoffWorld.x, wanted.y - foot.liftoffWorld.y, 0.0f }
                * (6.0f / (swing * swing));
        }
        if (progress < SwingFreeRevisionProgress) {
            path.committed = wanted;
        } else if (progress < SwingBoundedRevisionProgress) {
            if (!path.boundedFromValid) {
                path.boundedFrom = path.committed;
                path.boundedFromValid = true;
            }
            const Vec3 revision { wanted.x - path.boundedFrom.x, wanted.y - path.boundedFrom.y, 0.0f };
            if (revision.length() > SwingBoundedRevisionMeters && !path.boundedCounted) {
                ++m_plan.swingRevisionsBounded;
                path.boundedCounted = true;
            }
            const Vec3 bounded = clampMagnitude(revision, SwingBoundedRevisionMeters);
            path.committed = { path.boundedFrom.x + bounded.x, path.boundedFrom.y + bounded.y, wanted.z };
        } else if (std::hypot(wanted.x - path.committed.x, wanted.y - path.committed.y)
                > SwingRefusedRevisionMeters && !path.refusedCounted) {
            ++m_plan.swingRevisionsRefused;
            path.refusedCounted = true;
        }
        m_recoveryTarget[side] = path.committed;
        plannedPosition = m_recoveryTarget[side];
    } else if (foot.reason == StepReason3D::Brake) {
        // Freada: o alvo acompanha o corpo ate 70% do passo.
        if (progress < 0.7f)
            m_recoveryTarget[side] = brakeLanding(profile, input, side, plannedPosition, plannedOrientation);
        plannedPosition = m_recoveryTarget[side];
    } else if (foot.reason == StepReason3D::Locomotion) {
        // Andando pelo planejador: onde o corpo mantem a velocidade pedida;
        // o alvo acompanha o corpo ate 70% do passo.
        if (progress < 0.7f)
            m_recoveryTarget[side] = locomotionLanding(profile, input, side, plannedPosition, plannedOrientation);
        plannedPosition = m_recoveryTarget[side];
    } else if (foot.reason == StepReason3D::Landing) {
        // Descendo onde ia pousar, com o rumo que o pe ja tinha.
        plannedPosition = m_recoveryTarget[side];
        const float yaw = footYaw3D(link, foot.liftoffOrientationWorld);
        plannedOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
            yaw - footYaw3D(link, plannedOrientation)) * plannedOrientation).normalized();
    }
    // O pouso cabe inteiro num nivel (quina de degrau): o ponto anda o minimo
    // ao longo do pe, ate 10 cm; alem disso fica onde a pose pos.
    const Vec3 plannedForward = footForwardWorld3D(link, plannedOrientation);
    if (input.fitOnLevel) {
        const Vec3 moved { plannedPosition.x - m_fitCenter[side].x,
            plannedPosition.y - m_fitCenter[side].y, 0.0f };
        if (!m_fitValid[side] || moved.lengthSquared() > 0.0009f) {
            m_fitShift[side] = input.fitOnLevel(plannedPosition, plannedForward);
            if (m_fitShift[side].lengthSquared() > 0.01f) m_fitShift[side] = {};
            m_fitCenter[side] = plannedPosition;
            m_fitValid[side] = true;
        }
        plannedPosition += m_fitShift[side];
    }
    const FootSurface3D ground = input.surfaceAt
        ? input.surfaceAt(side, plannedPosition, plannedForward)
        : FootSurface3D { true, foot.liftoffGroundHeight, { 0.0f, 0.0f, 1.0f } };
    const Quaternion landing = (rotationBetween3D({ 0.0f, 0.0f, 1.0f }, soleNormal(ground))
        * plannedOrientation).normalized();
    Vec3 liveTarget = plannedPosition;
    liveTarget.z = footLockHeight3D(link, ground.height, landing);
    foot.goalWorld = liveTarget;
    foot.goalOrientationWorld = landing;
    foot.goalSurface = ground;

    Quaternion orientation = slerp3D(foot.liftoffOrientationWorld, landing, eased);
    const Vec3 ball = footBallLocal3D(link);
    const Vec3 from = foot.liftoffWorld;
    Vec3 stepTarget;
    float toeDown = 0.0f;
    const bool captureStepLift = (m_settings.recoveryCaptureTargeting
        && foot.reason == StepReason3D::Recovery)
        || (m_settings.locomotionStepping && foot.reason == StepReason3D::Locomotion)
        || foot.reason == StepReason3D::Brake;
    if (foot.path == SwingPath3D::Direct) {
        // Reto e baixo, desviando do pe de apoio se o caminho passa perto:
        // sempre para o lado deste pe (a direcao "do ponto mais perto" virava
        // ao meio quando o alvo passava ao lado do outro pe - o alvo saltava
        // ate 20 m/s num tick), e so enquanto o alvo esta ao longo dele.
        const bool continuous = m_settings.locomotionStepping && foot.reason == StepReason3D::Locomotion
            && m_swingTrajectory[side].valid && m_swingTrajectory[side].stepId == foot.stepId;
        if (continuous) {
            // No plano, a quintica ate o alvo comprometido no tempo que resta.
            SwingTrajectory3D& path = m_swingTrajectory[side];
            const Vec3 travel { liveTarget.x - from.x, liveTarget.y - from.y, 0.0f };
            advanceQuintic(path.position, path.velocity, path.acceleration, { liveTarget.x, liveTarget.y, 0.0f },
                travel * (-6.0f / (duration * duration)), duration - (foot.swingSeconds - deltaTime), deltaTime);
            stepTarget = { path.position.x, path.position.y, 0.0f };
        } else {
            stepTarget = from + (liveTarget - from) * eased;
        }
        if (supportsWeight(support.phase)) {
            const Vec3 outward = input.heading.rotate({ 0.0f, side == 0 ? 1.0f : -1.0f, 0.0f });
            const Vec3 forward = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 offset = stepTarget - support.anchorWorld;
            const float lateral = dot(offset, outward);
            const float along = std::abs(dot(offset, forward));
            const float near = 1.0f - smoothStep((along - 0.10f) / 0.15f);
            if (lateral < 0.14f)
                stepTarget += outward * ((0.14f - std::max(lateral, -0.14f)) * near * arch);
        }
        float lift = captureStepLift ? 0.08f : 0.045f;
        if (foot.reason == StepReason3D::Brake) {
            // Rapido, o pe vem de longe atras do corpo: mais alto, ou raspa.
            const float speed = std::hypot(input.planningComVelocityWorld.x,
                input.planningComVelocityWorld.y);
            lift = std::min(BrakeLiftMaximum,
                BrakeLift + BrakeLiftPerSpeed * speed);
            if (m_stumbleStep[side]) lift = std::max(lift, input.stumbleLiftMeters);
        }
        stepTarget.z = from.z + (liveTarget.z - from.z) * eased
            + (lift + foot.extraClearance) * arch;
        toeDown = 0.14f * std::sin(Pi * std::min(1.0f, progress * 1.4f));
    } else if (foot.path == SwingPath3D::Pivot) {
        // Abre girando sobre a bola do pe: ela quase nao sai do lugar, o
        // calcanhar sobe e roda em volta dela.
        const Vec3 ballFrom = from + foot.liftoffOrientationWorld.rotate(ball);
        const Vec3 ballTo = liveTarget + landing.rotate(ball);
        Vec3 ballNow = ballFrom + (ballTo - ballFrom) * eased;
        // O pivô ainda roda sobre a bola do pé, mas alivia a sola o bastante
        // para ela não raspar e segurar o giro inteiro.
        ballNow.z += (0.028f + foot.extraClearance) * arch;
        stepTarget = ballNow - orientation.rotate(ball);
        toeDown = 0.26f * arch;
    } else {
        // Contorna o pe de apoio em arco: angulo e distancia em volta dele,
        // nao a reta que passaria pela outra perna.
        const Vec3 pivot = support.anchorWorld;
        const Vec3 offsetFrom = from - pivot;
        const Vec3 offsetTo = liveTarget - pivot;
        const float fromRadius = std::hypot(offsetFrom.x, offsetFrom.y);
        const float toRadius = std::hypot(offsetTo.x, offsetTo.y);
        if (fromRadius > 0.05f && toRadius > 0.05f) {
            const float fromAngle = std::atan2(offsetFrom.y, offsetFrom.x);
            const float angle = fromAngle
                + wrapAngle(std::atan2(offsetTo.y, offsetTo.x) - fromAngle) * eased;
            const float radius = std::max(fromRadius + (toRadius - fromRadius) * eased,
                std::min(fromRadius, toRadius));
            stepTarget = pivot + Vec3 { std::cos(angle) * radius, std::sin(angle) * radius, 0.0f };
        } else {
            stepTarget = from + (liveTarget - from) * eased;
        }
        stepTarget.z = from.z + (liveTarget.z - from.z) * eased
            + ((foot.reason == StepReason3D::Turn ? 0.075f : 0.05f)
                + foot.extraClearance) * arch;
        // O calcanhar sai primeiro e o pe pousa plano.
        toeDown = 0.18f * std::sin(Pi * std::min(1.0f, progress * 1.4f));
    }
    if (toeDown > 0.0f) {
        // Ponta para baixo girando em volta da bola do pe.
        const Vec3 forward = footForwardWorld3D(link, orientation);
        Vec3 lateral = cross({ 0.0f, 0.0f, 1.0f }, forward);
        if (lateral.lengthSquared() > 0.0001f) {
            lateral = lateral.normalized();
            const Vec3 ballWorld = stepTarget + orientation.rotate(ball);
            orientation = (Quaternion::fromAxisAngle(lateral, toeDown) * orientation).normalized();
            stepTarget = ballWorld - orientation.rotate(ball);
        }
    }
    // Acabou o tempo sem contato: desce procurando o chao (degrau que nao
    // estava onde a sonda disse, chao mais baixo). Nunca apoia por tempo.
    const float overtime = foot.swingSeconds - duration;
    if (foot.phase == FootPhase3D::Swing && overtime >= 0.0f) {
        foot.phase = FootPhase3D::TouchdownSearch;
        ++m_plan.touchdownSearches;
    }
    if (foot.phase == FootPhase3D::TouchdownSearch) {
        stepTarget.z -= std::min(SearchDescentLimit, std::max(0.0f, overtime) * SearchDescentSpeed);
        m_plan.longestTouchdownSearchSeconds = std::max(m_plan.longestTouchdownSearchSeconds,
            std::max(0.0f, overtime));
    }
    foot.targetWorld = stepTarget;
    foot.targetOrientationWorld = orientation;

    // Contato: no fim do passo (ou procurando o chao), pouso. Depois de
    // metade, so se o pe ja saiu do chao neste passo e esta quase parado no
    // plano (o chao estava mais alto, um degrau): um pe que nunca saiu esta
    // raspando, nao pousando.
    const bool contact = contactSupport(measured);
    // Saiu do chao de verdade: alguns ticks sem contato E a sola acima do
    // chao de onde saiu (o contato de um pe arrastado tambem pisca).
    m_ticksWithoutContact[side] = contact ? 0 : m_ticksWithoutContact[side] + 1;
    if (m_ticksWithoutContact[side] >= 3
        && measured.soleLowestHeight > foot.liftoffGroundHeight + 0.01f) m_liftedOff[side] = true;
    const Vec3 soleVelocity = measured.soleVelocityWorld;
    const float planarSpeed = std::hypot(soleVelocity.x, soleVelocity.y);
    // Um pe arrastado (nunca saiu do chao) esta sempre em contato: ele pousa
    // quando chega perto do alvo, ou 0,25 s depois do tempo do passo - parar
    // onde estiver no fim do tempo deixava o pe na quina do degrau.
    const bool searching = foot.phase == FootPhase3D::TouchdownSearch;
    const PhysicsBodyState3D& body = input.state->links[m_feet[side]];
    // So pousa nivelado: a sola ate 1,5 cm acima do chao sondado sob o pe.
    // Tocando so a quina de um degrau (a sola 3-12 cm acima do chao sob o
    // pe), o passo continua ate o alvo encaixado no degrau - ou, 0,4 s depois
    // do tempo do passo, pousa assim mesmo.
    bool level = true;
    if (contact && input.surfaceAt && overtime < 0.40f) {
        const FootSurface3D under = input.surfaceAt(side, body.position,
            footForwardWorld3D(link, body.orientation));
        level = !under.valid || measured.soleLowestHeight <= under.height + 0.015f;
    }
    // (Na recuperacao, nao: o proximo passo espera este pousar.)
    // (Nem na marcha pelo planejador: esperando, o corpo seguia e deixava o
    // outro pe 80 cm para tras, esticado ate sair do chao.)
    const bool draggedDone = foot.reason == StepReason3D::Recovery
        || foot.reason == StepReason3D::Brake
        || (m_settings.locomotionStepping && foot.reason == StepReason3D::Locomotion)
        || std::hypot(body.position.x - foot.goalWorld.x,
            body.position.y - foot.goalWorld.y) < 0.04f || overtime >= 0.25f;
    // Recuperacao sem a ajuda (captura): contato antes do fim so e pouso perto
    // do alvo (8 cm) - no plano, um pe baixo raspando no meio do caminho
    // encerrava o passo curto e o corpo passava dele.
    const float toGoal = std::hypot(body.position.x - foot.goalWorld.x,
        body.position.y - foot.goalWorld.y);
    // (Tambem o passo da locomocao pelo planejador: raspando o chao no meio
    // do balanco ele pousava curto e o corpo passava dos pes.)
    const bool captureStep = (m_settings.recoveryCaptureTargeting
        && foot.reason == StepReason3D::Recovery)
        || (m_settings.locomotionStepping && foot.reason == StepReason3D::Locomotion)
        || foot.reason == StepReason3D::Brake;
    const bool earlyAllowed = !captureStep || toGoal < 0.08f || progress >= 0.9f;
    // Freada: tocando o chao depois de metade do passo ja a frente do centro
    // de massa, pousa onde tocou (o pe a frente ja freia; esperando o alvo
    // exato o corpo passava dele). Atras dele, raspando, nao: pousado ali o
    // pe nao freia nada.
    bool brakeTouch = false;
    if (foot.reason == StepReason3D::Brake && progress >= 0.5f) {
        Vec3 direction = m_stumbleStep[side] ? input.stumbleDirectionWorld : input.brakeDirectionWorld;
        direction.z = 0.0f;
        const float ahead = direction.lengthSquared() > 1e-4f
            ? dot(Vec3 { measured.soleCenterWorld.x - input.physical->centerOfMassWorld.x,
                measured.soleCenterWorld.y - input.physical->centerOfMassWorld.y, 0.0f }, direction.normalized())
            : 1.0f;
        // (So se o alvo ainda estava mais a frente: tocando adiante do alvo
        // - o passo que fecha a base atras do ponto de captura -, o pe segue
        // ate ele; pousado ali, o corpo parava atras dos dois pes.)
        const float shortOfGoal = direction.lengthSquared() > 1e-4f
            ? dot(Vec3 { foot.goalWorld.x - measured.soleCenterWorld.x,
                foot.goalWorld.y - measured.soleCenterWorld.y, 0.0f }, direction.normalized())
            : 1.0f;
        brakeTouch = (ahead > BrakeTouchAhead && shortOfGoal > -0.03f) || progress >= 0.9f;
    }
    // O pivo (giro sobre a bola do pe) quase nao sai do chao por construcao:
    // ele pousa pelo contato no fim do passo como os outros. Esperando o pe
    // "sair do chao", ele so terminava 0,25 s depois do tempo, contado como
    // arrastado, e o proximo passo daquele pe esperava cada vez mais: cada
    // passo de giro levava ~0,6 s.
    const bool lifted = m_liftedOff[side] || foot.path == SwingPath3D::Pivot;
    if (contact && level && ((searching && (lifted || draggedDone))
            || (lifted && brakeTouch)
            || (lifted && earlyAllowed && (progress >= LateTouchdownProgress
                || (progress >= EarlyTouchdownProgress && planarSpeed < TouchdownPlanarSpeed))))) {
        land(profile, input, side);
        return;
    }
    // Esbarrou em algo de lado no caminho: o arco sobe.
    if (measured.touching && !measured.supporting && progress < 0.8f
        && foot.extraClearance < BlockedExtraClearance) {
        foot.extraClearance = BlockedExtraClearance;
        ++m_plan.blockedSwings;
    }
}

void ContactFootwork3D::update(const RagdollProfile3D& profile,
    const ContactFootworkInput3D& rawInput, float deltaTime) {
    if (!m_initialized || m_linkCount != profile.links.size()) reset(profile);
    // Giro parado com a ajuda cheia: enquanto gira (e um instante depois) a
    // base de referencia e a compacta - pes lado a lado na largura do
    // quadril, pouco abertos, no olhar. Pela pose parada em alerta (rodada
    // ~22 graus, escalonada, o pe de tras ~0,6 m e 46 graus aberto), cada
    // passo de giro varria um arco enorme e virava afundo, pernas abertas e
    // joelhos tortos. Tudo no planejador (torcao da base, alvos, acomodacao)
    // usa a mesma base.
    ContactFootworkInput3D input = rawInput;
    // Pes roteirizados: nenhum passo do planejador (o clipe decide).
    if (rawInput.scripted[0].active || rawInput.scripted[1].active) input.allowSteps = false;
    {
        const float ahead = std::abs(wrapAngle(rawInput.standingTurnTargetYawRadians - rawInput.facingYawRadians));
        bool turningFeet = false;
        for (const FootPlan3D& foot : m_plan.feet)
            turningFeet = turningFeet || (foot.reason == StepReason3D::Turn && !foot.followingGait
                && foot.phase != FootPhase3D::Stance);
        // (Tambem quando o rumo se afastou de onde a base assentou: a camera
        // girando devagar acumula o giro sem pedir nada de uma vez.)
        const float drift = std::abs(wrapAngle(rawInput.facingYawRadians - m_stanceFacing));
        const bool active = rawInput.standing && (ahead > TurnModeEnter || drift > TurnModeDrift
            || std::abs(rawInput.standingTurnRateRadiansPerSecond) > TurnModeRate || turningFeet);
        if (std::getenv("XTURNMODEDBG") && active && m_turnModeSeconds > 0.2f)
            std::printf("TURNMODE on | ahead %.2f drift %.2f rate %.2f feet %d standing %d\n", ahead, drift,
                rawInput.standingTurnRateRadiansPerSecond, turningFeet ? 1 : 0, rawInput.standing ? 1 : 0);
        m_turnModeSeconds = active ? 0.0f : std::min(10.0f, m_turnModeSeconds + deltaTime);
        m_turnMode = rawInput.assistHold > 0.5f && m_turnModeSeconds < TurnModeHold;
        // Giro parado pedido ou em andamento, com qualquer ajuda.
        m_turnRequested = active;
        // Depois de um giro a base compacta fica ate ele andar: voltar para
        // a pose parada em alerta era um passo largo para tras (~0,6 m) 1-2 s
        // depois, parecendo afundo.
        // A base quadrada vale so girando (e o instante depois); parado, a
        // base em alerta volta com uma acomodacao.
        m_compactStance = m_turnMode && TurnStanceHalfWidth > 0.0f;
        // A base assenta no rumo de quem anda ou gira; parado, fica.
        if (!rawInput.standing || m_turnMode) m_stanceFacing = rawInput.facingYawRadians;
        // Cada giro libera de novo uma acomodacao (a volta para a base parada).
        if (m_turnMode) m_heldSettles = 0;
        if (m_turnMode && TurnStanceScale < 0.999f && m_feet[0] < profile.links.size() && m_feet[1] < profile.links.size()) {
            // Girando: a base de alerta encurtada em volta do centro dela -
            // firme e escalonada, sem o pe de tras 0,5 m atras varrendo o
            // chao (afundo). Ao parar de girar, uma acomodacao curta devolve a
            // base inteira.
            const Vec3 center = (rawInput.reference[0].poseWorld + rawInput.reference[1].poseWorld) * 0.5f;
            for (std::size_t side = 0; side < 2; ++side) {
                FootReference3D& reference = input.reference[side];
                const float z = reference.poseWorld.z;
                reference.poseWorld = center + (rawInput.reference[side].poseWorld - center) * TurnStanceScale;
                reference.poseWorld.z = z;
            }
        }
        if (m_compactStance && m_feet[0] < profile.links.size() && m_feet[1] < profile.links.size()) {
            // A base do giro alinhada com a pelve (a pose em alerta a roda ~22
            // graus do olhar): de frente para o olhar, a perna ja comecava
            // torcida e chegava a ~75 graus girando para a direita.
            const float stanceYawNow = rawInput.facingYawRadians + rawInput.pelvisYawOffsetRadians;
            const Vec3 forward { std::cos(stanceYawNow), std::sin(stanceYawNow), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            float along = 0.0f;
            for (std::size_t side = 0; side < 2; ++side)
                along += 0.5f * dot(rawInput.reference[side].poseWorld - rawInput.referenceRootWorld, forward);
            const Vec3 center = Vec3 { rawInput.referenceRootWorld.x, rawInput.referenceRootWorld.y, 0.0f }
                + forward * along;
            for (std::size_t side = 0; side < 2; ++side) {
                const float outward = side == 0 ? 1.0f : -1.0f;
                FootReference3D& reference = input.reference[side];
                const float z = reference.poseWorld.z;
                reference.poseWorld = center + left * (outward * TurnStanceHalfWidth)
                    + forward * (outward * TurnStanceStagger);
                reference.poseWorld.z = z;
                const float yaw = stanceYawNow + outward * TurnStanceToeOut;
                const float current = footYaw3D(profile.links[m_feet[side]], reference.poseOrientationWorld);
                reference.poseOrientationWorld = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                    wrapAngle(yaw - current)) * reference.poseOrientationWorld).normalized();
            }
        }
        m_plan.turnStance = m_compactStance;
    }
    CharacterContactPlan3D& plan = m_plan;
    plan.valid = false;
    plan.supportShiftWorld = {};
    plan.recoverySide = -1;
    plan.recoveryUrgency = 0.0f;
    plan.plannedOutsideMeters = 0.0f;
    if (input.physical == nullptr || !input.physical->valid || input.state == nullptr
        || input.state->links.size() != profile.links.size()
        || m_feet[0] >= profile.links.size() || m_feet[1] >= profile.links.size()
        || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
        return;
    }
    const CharacterPhysicalState3D& physical = *input.physical;
    plan.valid = true;

    // Carga de cada pe (fracao do peso), filtrada ~30 ms. Com um pe so
    // apoiando, ele tem tudo. Com os dois: se o impulso veio do solver
    // (PhysX), a medida; se veio estimado antes do solver (Jolt), a alavanca
    // do centro de massa entre as solas - o impulso estimado so separa "no
    // ar" de "no chao" (medido: 0,50/0,50 o tempo todo com a pelve indo 5 cm
    // para um lado).
    {
        const FootSupportEstimate3D& left = physical.feet[0];
        const FootSupportEstimate3D& right = physical.feet[1];
        float share = plan.loadShare[0];
        bool known = true;
        if (left.supporting && right.supporting) {
            const bool solved = left.evidence == ContactEvidence3D::SolvedImpulse
                && right.evidence == ContactEvidence3D::SolvedImpulse;
            const float total = left.loadNewtons + right.loadNewtons;
            if (solved && total > 1.0f) {
                share = left.loadNewtons / total;
            } else {
                Vec3 axis = right.soleCenterWorld - left.soleCenterWorld;
                axis.z = 0.0f;
                // Marcha E5: pelo centro de pressao (ZMP) da aceleracao
                // observada do COM, nao pelo COM: andando o corpo acelera, e o
                // COM entre as solas dizia pouco (~3 passos por corrida saiam
                // "carregados"). Ignora o momento angular (proxy, nao medida).
                Vec3 point = physical.centerOfMassWorld;
                if (m_settings.locomotionStepping && m_settings.zmpLoadEstimate) {
                    const Vec3 a = physical.centerOfMassAccelerationWorld;
                    const float height = std::max(0.3f, physical.centerOfMassWorld.z
                        - std::min(left.soleCenterWorld.z, right.soleCenterWorld.z));
                    const float vertical = std::max(2.0f, 9.81f + a.z);
                    point.x -= a.x * height / vertical;
                    point.y -= a.y * height / vertical;
                }
                Vec3 com = point - left.soleCenterWorld;
                com.z = 0.0f;
                const float length = axis.lengthSquared();
                share = length > 1e-4f
                    ? 1.0f - std::clamp(dot(com, axis) / length, 0.0f, 1.0f) : 0.5f;
            }
        } else if (left.supporting) {
            share = 1.0f;
        } else if (right.supporting) {
            share = 0.0f;
        } else {
            known = false;
        }
        if (known) {
            const float blend = 1.0f - std::exp(-deltaTime / 0.03f);
            plan.loadShare[0] += (share - plan.loadShare[0]) * blend;
            plan.loadShare[1] = 1.0f - plan.loadShare[0];
        }
    }

    // Freando com os dois pes da passada no ar (a troca caiu no fim de um
    // balanco, o outro pe ja descendo): so um da o passo de freada - o mais
    // alto; o que ja ia tocar o chao pousa onde ia. Os dois em passo de
    // freada deixavam o corpo sem apoio por ~0,15 s e a ajuda o erguia
    // (saltinho e queda parando de um strafe a 3 m/s).
    int brakeLandingSide = -1;
    if (BrakeSingleTakeoverStep && input.braking && input.allowSteps && !input.seedFromBody
        && plan.feet[0].followingGait && plan.feet[1].followingGait) {
        std::array<bool, 2> loose {};
        std::array<float, 2> height {};
        for (std::size_t side = 0; side < 2; ++side) {
            const FootSupportEstimate3D& seen = physical.feet[side];
            loose[side] = !input.gait[side].planted && !(contactSupport(seen)
                && std::hypot(seen.soleVelocityWorld.x, seen.soleVelocityWorld.y) <= 0.50f);
            height[side] = seen.soleLowestHeight - (input.gait[side].surface.valid
                ? input.gait[side].surface.height : plan.feet[side].surface.height);
        }
        if (loose[0] && loose[1]) brakeLandingSide = height[0] <= height[1] ? 0 : 1;
    }
    // Quem decide cada pe: parado (ou num passo daqui em andamento), o
    // planejador; andando, no ar, caindo ou levantando, a passada/os
    // reflexos, e o plano so acompanha.
    for (std::size_t side = 0; side < 2; ++side) {
        FootPlan3D& foot = plan.feet[side];
        // Um passo daqui ja com o pe no ar (ou pousando) segue daqui ate
        // carregar; na descarga o pe ainda esta no chao - andando, a passada
        // assume dali.
        // Pousos de troca (so trazer o pe ao chao) voltam para a passada
        // assim que ela retoma: nao sao passos decididos aqui.
        const bool handoverLanding = foot.reason == StepReason3D::Landing
            || foot.reason == StepReason3D::ReestablishSupport;
        const bool stepInFlight = !foot.followingGait && foot.phase != FootPhase3D::Stance
            && foot.phase != FootPhase3D::Unloading && !handoverLanding;
        const bool own = input.allowSteps && (input.standing || input.braking || input.stumbling || stepInFlight
            || (m_settings.locomotionStepping && input.locomoting));
        if (!own) {
            const GaitFootState3D& gait = input.gait[side];
            foot.followingGait = true;
            foot.phase = gait.planted ? FootPhase3D::Stance : FootPhase3D::Swing;
            foot.reason = gait.planted ? StepReason3D::None : StepReason3D::Locomotion;
            if (gait.planted) {
                foot.anchorWorld = gait.lockedWorld;
                foot.anchorOrientationWorld = gait.lockedOrientationWorld;
                foot.surface = gait.surface;
            }
            continue;
        }
        if (!foot.followingGait) continue;
        // Assume o pe: apoiado na passada, fica onde esta travado; no ar,
        // vai dali (medido) ate a base parada num passo que so pousa por
        // contato.
        foot.followingGait = false;
        const GaitFootState3D& gait = input.gait[side];
        if (gait.planted && !input.seedFromBody) {
            foot.phase = FootPhase3D::Stance;
            foot.reason = StepReason3D::None;
            foot.anchorWorld = gait.lockedWorld;
            foot.anchorOrientationWorld = gait.lockedOrientationWorld;
            foot.surface = gait.surface;
        } else if (!input.seedFromBody && input.stumbling && !input.braking) {
            // Quase-queda: o pe da passada no chao (de fato) fica; dos que
            // estao no ar, so o mais a frente no rumo da o passo que pega o
            // corpo (o outro desce onde esta). Com os dois em passo, o de tras
            // mirava a frente a partir do outro, tambem atras, e o alvo ficava
            // atras do corpo.
            const std::size_t other = 1 - side;
            Vec3 direction = input.stumbleDirectionWorld;
            direction.z = 0.0f;
            const bool otherAir = !(contactSupport(physical.feet[other]) && physical.feet[other].soleLowestHeight < 0.04f)
                && !input.gait[other].planted;
            const bool otherAhead = otherAir && direction.lengthSquared() > 1e-4f
                && dot(physical.feet[other].soleCenterWorld - physical.feet[side].soleCenterWorld, direction) > 0.0f;
            if ((contactSupport(physical.feet[side]) && physical.feet[side].soleLowestHeight < 0.04f)
                || otherAhead || m_stumbleStepsTaken > 0) {
                seedFoot(profile, input, side);
            } else {
                m_stumbleStep[side] = true;
                startStep(profile, input, side, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
                release(profile, input, side, ReleaseReason3D::Recovery);
                m_brakeClosing[side] = false;
                m_brakeRearLanding[side] = false;
                m_liftedOff[side] = true;
                foot.swingDurationSeconds = input.stumbleSwingSeconds;
                m_recoveryTarget[side] = foot.liftoffWorld;
                ++m_stumbleStepsTaken;
            }
        } else if (!input.seedFromBody && input.braking && contactSupport(physical.feet[side])
            && std::hypot(physical.feet[side].soleVelocityWorld.x, physical.feet[side].soleVelocityWorld.y) <= 0.50f) {
            // Freando, o pe que ja apoia de fato (o contato - o clipe nem
            // sempre o marca plantado) fica onde esta: e o apoio que freia.
            // Assumido como "no ar", ele tambem dava um passo de freada e a
            // parada tinha sempre dois.
            seedFoot(profile, input, side);
        } else if (!input.seedFromBody && input.braking && static_cast<int>(side) == brakeLandingSide) {
            seedFoot(profile, input, side);
        } else if (!input.seedFromBody && input.braking && SidewaysInnerLands && [&] {
                // Freando de lado, o pe de dentro (o oposto ao rumo) no ar so
                // toca onde esta - o passo de freada e do de fora, que abre.
                // Como passo de freada ele nao podia cruzar e pousava longe,
                // do lado oposto a queda; o de fora so abria depois, com o
                // corpo ja alem dele (parando de um strafe a 5%: 14 em 16).
                Vec3 direction = input.brakeDirectionWorld;
                direction.z = 0.0f;
                if (direction.lengthSquared() < 1e-4f) return false;
                const float sideways = dot(direction.normalized(), input.heading.rotate({ 0.0f, 1.0f, 0.0f }));
                if (std::abs(sideways) <= BrakeLateral) return false;
                return side == (sideways > 0.0f ? 1u : 0u);
            }()) {
            seedFoot(profile, input, side);
        } else if (!input.seedFromBody && input.braking) {
            // Freando: o pe da passada que esta no ar ja vai para o pouso de
            // freada, de onde esta. (Como "refazer a base" - 0,34 s ate a pose
            // parada - nenhum passo de freada saia antes de ele pousar, e o
            // corpo passava dos dois pes.)
            startStep(profile, input, side, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
            release(profile, input, side, ReleaseReason3D::Recovery);
            m_brakeClosing[side] = false;
            ++m_brakeStepsTaken;
            // Se o outro pe ja esta plantado a frente do corpo freando, este
            // desce logo atras do corpo, como o apoio de tras do afundo (a
            // referencia Run To Stop: um pe planta e freia, o outro fica). Ir
            // tambem a frente deixava os dois adiante, ele freava demais,
            // parava antes dos pes e vinham passos de correcao.
            {
                Vec3 direction = input.brakeDirectionWorld;
                direction.z = 0.0f;
                const std::size_t other = 1 - side;
                const FootSupportEstimate3D& seen = physical.feet[other];
                m_brakeRearLanding[side] = direction.lengthSquared() > 1e-4f && seen.supporting
                    && dot(Vec3 { seen.soleCenterWorld.x - physical.centerOfMassWorld.x,
                        seen.soleCenterWorld.y - physical.centerOfMassWorld.y, 0.0f },
                        direction.normalized()) > BrakeRearPlantAhead;
            }
            m_liftedOff[side] = true;
            foot.swingDurationSeconds = BrakeTakeoverSwingShare * BrakeSwingSeconds;
            m_recoveryTarget[side] = foot.liftoffWorld;
        } else if (!input.seedFromBody) {
            seedFoot(profile, input, side);
        }
    }

    // Refazer a base a partir do corpo (fim do levantar): pe rente ao chao
    // vira apoio onde esta (nunca puxar uma perna erguida para baixo nem
    // mover o corpo simulado na troca); pe erguido vai para a base num passo
    // de onde ele esta, com arco.
    if (input.seedFromBody && input.allowSteps) {
        for (std::size_t side = 0; side < 2; ++side) seedFoot(profile, input, side);
    }

    const bool locomotionPlanning = m_settings.locomotionStepping && input.locomoting
        && !input.standing;
    const bool planning = (input.standing || locomotionPlanning || input.braking || input.stumbling) && input.allowSteps
        && !plan.feet[0].followingGait && !plan.feet[1].followingGait;
    if (!input.stumbling) {
        m_stumbleStepsTaken = 0;
        m_stumbleStep = {};
        m_stumbleSeconds = 0.0f;
    } else {
        m_stumbleSeconds += deltaTime;
    }
    if (!input.standing) m_pendingTurnFoot = -1;

    // Precisa de passo: o ponto de captura fora do apoio PLANEJADO - os pes
    // no chao onde estao e o pe em passo onde vai pousar (no meio de um passo
    // comum o apoio e um pe so, e o ponto "sair" dele e normal). O pe do lado
    // da queda (de lado), ou o de tras em relacao a ela (frente/tras), da o
    // passo.
    // (Andando pelo planejador, o proprio lugar do pouso e a recuperacao.)
    // O ponto de captura fora do apoio PLANEJADO: os pes no chao onde estao
    // e o pe em passo onde vai pousar.
    const auto plannedCaptureOutside = [&]() {
        std::vector<std::array<float, 2>> planned;
        for (std::size_t side = 0; side < 2; ++side) {
            const FootPlan3D& foot = plan.feet[side];
            const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
            const bool inAir = airborne(foot.phase);
            const Vec3 at = inAir ? foot.goalWorld : foot.anchorWorld;
            const Quaternion orientation = inAir ? foot.liftoffOrientationWorld
                : foot.anchorOrientationWorld;
            const Vec3 half = ragdollColliderHasBox3D(link.collider)
                ? link.collider.boxHalfExtents
                : Vec3 { link.collider.radiusMeters, link.collider.radiusMeters,
                    link.collider.radiusMeters };
            for (const float sx : { -1.0f, 1.0f }) {
                for (const float sy : { -1.0f, 1.0f }) {
                    const Vec3 corner = at + orientation.rotate(link.collider.localPosition
                        + link.collider.localOrientation.rotate({ sx * half.x, sy * half.y, 0.0f }));
                    planned.push_back({ corner.x, corner.y });
                }
            }
        }
        return planarDistanceOutsideHull3D(planned, input.capturePointWorld.x, input.capturePointWorld.y);
    };
    if (planning && !locomotionPlanning && !input.braking && !input.stumbling && input.allowRecovery) {
        plan.plannedOutsideMeters = plannedCaptureOutside();
        // Com um pe no ar, vale o apoio planejado; com os dois no chao, tambem
        // o medido (a perna nem sempre alcanca o alvo travado, e o planejado
        // escondia isso).
        const bool stepping = airborne(plan.feet[0].phase) || airborne(plan.feet[1].phase);
        // (Sem a ajuda, com um passo comum no ar o apoio planejado inclui o
        // alvo dele - que segue o corpo -, e um empurrao no meio de um passo
        // de acomodacao nao virava recuperacao: o passo seguia no ritmo
        // lento e o primeiro passo de recuperacao saia 0,33 s depois. O
        // medido vale tambem, com a folga normal de um pe so, 12 cm.)
        const float measuredWhileStepping = m_settings.recoveryCaptureTargeting
            ? input.captureOutsideMeters - 0.12f : 0.0f;
        // Parando de uma freada (sem nada empurrando), com os dois pes no
        // plano de apoio, o apoio medido conta tambem o pe rente ao chao (a
        // sola ate 3 cm acima dele) sem contato: o corpo balancando sobre um
        // pe tira o outro do chao por um instante, a base medida virava um pe
        // so e saia um passo sem queda nenhuma (um ou dois a cada parada).
        // (Para tudo, os empurroes continuos de 0,6-1 m/s caiam mais: 42 ->
        // 45-48 quedas em 216.)
        float measuredOutside = input.captureOutsideMeters;
        if (!stepping && input.quietStop && RecoveryNearGround > 0.0f
            && physical.externalForceWorld.length() < RecoveryForceTrigger) {
            std::vector<std::array<float, 2>> near;
            for (std::size_t side = 0; side < 2; ++side) {
                const FootSupportEstimate3D& foot = physical.feet[side];
                if (!foot.supporting && foot.soleLowestHeight
                        > plan.feet[side].surface.height + RecoveryNearGround) continue;
                for (const Vec3& corner : foot.soleCornersWorld) near.push_back({ corner.x, corner.y });
            }
            if (!near.empty())
                measuredOutside = planarDistanceOutsideHull3D(near,
                    input.capturePointWorld.x, input.capturePointWorld.y);
        }
        const float outside = stepping
            ? std::max(plan.plannedOutsideMeters, measuredWhileStepping)
            : std::max(plan.plannedOutsideMeters, measuredOutside);

        // So com o corpo indo para fora do apoio (ou sob forca externa):
        // parado com o centro de massa um pouco fora de uma base estreita (o
        // giro no lugar, com os pes torcidos) nao e queda.
        Vec3 bodyVelocity = input.planningComVelocityWorld;
        bodyVelocity.z = 0.0f;
        const float forceSize = physical.externalForceWorld.length();
        const bool leaving = dot(bodyVelocity, input.captureDirectionWorld) > RecoveryLeavingSpeed
            || forceSize > RecoveryForceTrigger;
        // Ajuda cheia segurando: so com forca de fora ou um desvio grande
        // (depois de parar, o corpo segurado ainda deixava a captura fora
        // da base: 2-4 passos de recuperacao a cada parada, sem queda
        // nenhuma por perto).
        // Girando parado (passos de giro em andamento ou o modo de giro),
        // sem forca de fora: um pe no ar tira a captura da base sem queda
        // nenhuma - virava recuperacao, transformava o passo de giro e
        // segurava os seguintes 0,45 s (180 graus levavam 2-3 s, a pelve
        // presa no limite de torcao esperando).
        bool turning = m_turnMode || m_turnRequested;
        for (const FootPlan3D& foot : plan.feet)
            turning = turning || (foot.reason == StepReason3D::Turn && !foot.followingGait
                && foot.phase != FootPhase3D::Stance);
        turning = turning || m_pendingTurnFoot >= 0;
        const float holdMargin = forceSize > RecoveryForceTrigger ? 0.0f
            : std::max(AssistHoldMargin * std::clamp(input.assistHold, 0.0f, 1.0f),
                turning ? TurnRecoveryMargin : 0.0f);
        if (outside > RecoveryOutsideTrigger + holdMargin && leaving) {
            plan.recoveryUrgency = smoothStep((outside - RecoveryOutsideTrigger) / RecoveryOutsideFull);
            Vec3 fall = input.captureDirectionWorld;
            if (fall.lengthSquared() < 1e-4f && forceSize > 1.0f)
                fall = physical.externalForceWorld * (1.0f / forceSize);
            const Vec3 forward = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 left = input.heading.rotate({ 0.0f, 1.0f, 0.0f });
            const float sideways = dot(fall, left);
            if (std::abs(sideways) > 0.5f * std::abs(dot(fall, forward))) {
                // De lado: o pe do lado da queda abre; com a base ja aberta
                // (ele mais de 30 cm para fora), o outro fecha.
                const std::size_t lead = sideways > 0.0f ? 0 : 1;
                const Vec3 outward = left * (sideways > 0.0f ? 1.0f : -1.0f);
                const float spread = dot(plan.feet[lead].anchorWorld
                    - plan.feet[1 - lead].anchorWorld, outward);
                plan.recoverySide = static_cast<int>(spread > 0.30f ? 1 - lead : lead);
                // Sem a ajuda (captura): de lado os pes se alternam - o de fora
                // abre, o de dentro fecha, o de fora abre de novo. Repetindo o
                // de fora, carregado (o corpo vai sobre ele), ele so arrastava.
                if (m_settings.recoveryCaptureTargeting && m_lastRecoverySide == static_cast<int>(lead)
                    && m_sinceRecoveryLanding < 0.6f)
                    plan.recoverySide = static_cast<int>(1 - lead);
            } else {
                const float leftAlong = dot(plan.feet[0].anchorWorld, fall);
                const float rightAlong = dot(plan.feet[1].anchorWorld, fall);
                plan.recoverySide = leftAlong < rightAlong ? 0 : 1;
                // Sem a ajuda (captura): se o corpo ja virou sobre um pe e o
                // outro saiu do chao, o passo e do pe livre - o de tras,
                // carregado, so arrastava (empurrao por tras).
                // (Livre de verdade: sola 2 cm acima do chao dele - o pe que
                // acabou de pousar pisca sem apoio e era escolhido de novo,
                // carregado.)
                const auto lifted = [&](std::size_t side) {
                    return !physical.feet[side].supporting
                        && physical.feet[side].soleLowestHeight > plan.feet[side].surface.height + 0.02f;
                };
                if (m_settings.recoveryCaptureTargeting && lifted(0) != lifted(1)
                    && physical.feet[lifted(0) ? 1 : 0].supporting)
                    plan.recoverySide = lifted(0) ? 0 : 1;
            }
        }
    }
    if (!input.standing || input.assistHold <= 0.5f) m_heldSettles = 0;
    m_sinceRecoveryNeeded = plan.recoverySide >= 0 ? 0.0f
        : std::min(10.0f, m_sinceRecoveryNeeded + deltaTime);
    plan.recovering = m_sinceRecoveryNeeded < RecoveryMemorySeconds;

    for (float& cooldown : m_stepCooldown) cooldown = std::max(0.0f, cooldown - deltaTime);
    m_sinceRecoveryLanding = std::min(10.0f, m_sinceRecoveryLanding + deltaTime);
    m_sinceLocomotionTouchdown = std::min(10.0f, m_sinceLocomotionTouchdown + deltaTime);
    // Locomocao pelo planejador (E5): um pe depois do outro, sem esperar a
    // passada do clipe - pousado um (por contato), depois de uma dupla
    // sustentacao curta sai o outro. O primeiro e o de tras em relacao ao
    // rumo pedido.
    if (planning && locomotionPlanning) {
        bool anyAirborne = false, anyUnloading = false;
        for (const FootPlan3D& foot : plan.feet) {
            anyAirborne = anyAirborne || airborne(foot.phase);
            anyUnloading = anyUnloading || foot.phase == FootPhase3D::Unloading;
        }
        Vec3 desired = input.desiredVelocityWorld;
        desired.z = 0.0f;
        const float speed = desired.length();
        // (So depois de o pe que pousou aceitar o apoio - fase 5.)
        const bool anyLoading = m_settings.loadAcceptanceByEvent
            && (plan.feet[0].phase == FootPhase3D::Loading
                || plan.feet[1].phase == FootPhase3D::Loading);
        if (!anyAirborne && !anyUnloading && !anyLoading
            && m_sinceLocomotionTouchdown >= LocomotionDoubleSupportSeconds
            && speed > LocomotionMinimumSpeed) {
            std::size_t side = 0;
            if (m_lastLocomotionSide >= 0) {
                side = static_cast<std::size_t>(1 - m_lastLocomotionSide);
            } else {
                // Arrancada: para a frente/tras sai o pe de tras em relacao ao
                // rumo; de lado, o pe do lado para onde vai (o corpo cai para
                // ele e o passo abre - saindo o de tras, ele ia para o lado
                // errado).
                const Vec3 direction = desired * (1.0f / speed);
                const Vec3 left = input.heading.rotate({ 0.0f, 1.0f, 0.0f });
                const Vec3 forward = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
                const float sideways = dot(direction, left);
                if (std::abs(sideways) > std::abs(dot(direction, forward)))
                    side = sideways > 0.0f ? 0 : 1;
                else
                    side = dot(plan.feet[0].anchorWorld, direction) < dot(plan.feet[1].anchorWorld, direction) ? 0 : 1;
            }
            startStep(profile, input, side, StepReason3D::Locomotion, SwingPath3D::Direct, 0.0f);
            plan.feet[side].swingDurationSeconds = locomotionSwingSeconds(speed);
        }
    }
    const auto ready = [&](std::size_t side) { return m_stepCooldown[side] <= 0.0f; };
    if (m_settings.benchTransferFoot >= 0 && input.standing && input.allowSteps) {
        const std::size_t side = static_cast<std::size_t>(std::clamp(m_settings.benchTransferFoot, 0, 1));
        if (plan.feet[side].phase == FootPhase3D::Stance && plan.feet[1 - side].phase == FootPhase3D::Stance
            && !plan.feet[side].followingGait && !plan.feet[1 - side].followingGait) {
            startStep(profile, input, side, StepReason3D::Locomotion, SwingPath3D::Direct, 0.0f);
            m_benchHolding[side] = true;
        }
    }
    // Freada: um passo de cada vez, sem voo - o pe de tras sai assim que o
    // outro apoia, e so enquanto o ponto de captura ainda passa do pe da
    // frente (senao o corpo ja para sobre a base que tem).
    plan.brakeSettled = false;
    plan.brakeYield = false;
    if (!input.braking) m_brakeStepsTaken = 0;
    // No maximo BrakeMaximumSteps passos por freada (o pedido do usuario:
    // freada imediata, 1-2 passos se precisar): depois deles a freada acaba e
    // o equilibrio de parado cuida do resto. (De lado, sem limite, o
    // arrasta-pe chegava a 12 passos.)
    if (planning && input.braking && m_brakeStepsTaken >= BrakeMaximumSteps) {
        bool busy = false;
        for (const FootPlan3D& foot : plan.feet)
            busy = busy || airborne(foot.phase) || foot.phase == FootPhase3D::Unloading;
        plan.brakeSettled = !busy;
        // (E entrega na hora, ainda que o corpo nao tenha parado: esperando
        // ele ficar devagar, com a recuperacao desligada, um corpo
        // escorregando de lado nunca ficava e caia.)
        plan.brakeYield = !busy;
    } else if (planning && input.braking) {
        bool busy = false;
        for (const FootPlan3D& foot : plan.feet)
            busy = busy || airborne(foot.phase) || foot.phase == FootPhase3D::Unloading;
        Vec3 direction = input.brakeDirectionWorld;
        direction.z = 0.0f;
        if (!busy && direction.lengthSquared() > 1e-4f) {
            direction = direction.normalized();
            const std::size_t trailing = dot(plan.feet[0].anchorWorld, direction)
                < dot(plan.feet[1].anchorWorld, direction) ? 0 : 1;
            const std::size_t leading = 1 - trailing;
            const Vec3 front = physical.feet[leading].supporting ? physical.feet[leading].soleCenterWorld
                : plan.feet[leading].anchorWorld;
            const float captureAhead = dot(Vec3 { input.capturePointWorld.x - front.x,
                input.capturePointWorld.y - front.y, 0.0f }, direction);
            // Passo de fechamento: o corpo ja para sobre o pe da frente, mas a
            // base ficou comprida (o de tras 40 cm ou mais atras) - o de tras
            // vem para perto do da frente antes de assentar. Parado assim, o
            // primeiro giro ou acomodacao tirava o pe de tras com o peso nele.
            const Vec3 rear = physical.feet[trailing].supporting ? physical.feet[trailing].soleCenterWorld
                : plan.feet[trailing].anchorWorld;
            const float stance = dot(Vec3 { front.x - rear.x, front.y - rear.y, 0.0f }, direction);
            const float speed = std::hypot(input.planningComVelocityWorld.x, input.planningComVelocityWorld.y);
            const bool closing = stance > BrakeCloseStance && speed < 1.5f;
            // O ponto de captura atras do pe de tras (o corpo parando antes dos
            // pes, os dois adiante dele): sem passo ele cai para tras e a
            // recuperacao de parado dava dois ou tres passos para tras depois;
            // aqui sai um so, logo, para tras do ponto de captura.
            const float overRear = dot(Vec3 { input.capturePointWorld.x - rear.x,
                input.capturePointWorld.y - rear.y, 0.0f }, direction);
            // (So atras de fato: com uma margem a frente do pe, o mesmo pe dava
            // cinco passos para o mesmo lugar, cada um desequilibrando o corpo.)
            const bool behind = overRear < -BrakeSettleMargin && captureAhead < 0.0f;
            plan.brakeSettled = !behind && captureAhead <= BrakeStepTrigger && !closing;
            // O ponto de captura saiu da base para o lado do rumo (o corpo
            // escorrega de lado enquanto freia): a freada nao tem passo para
            // isso e a recuperacao, desligada enquanto ela dura, nao agia - o
            // corpo derivava a 0,5-0,8 m/s de lado e caia. Ela entrega.
            const float outsideBase = plannedCaptureOutside();
            const float sideways = dot(direction, input.heading.rotate({ 0.0f, 1.0f, 0.0f }));
            if (std::abs(sideways) > BrakeLateral) {
                // De lado: o pe do lado do movimento abre (o passo de freada) e
                // o outro so fecha a base, sem cruzar - mandar o de tras por
                // cima do da frente cruzava as pernas e ficava sem alcance.
                const std::size_t outer = sideways > 0.0f ? 0 : 1;
                const std::size_t inner = 1 - outer;
                const Vec3 outerAt = physical.feet[outer].supporting ? physical.feet[outer].soleCenterWorld
                    : plan.feet[outer].anchorWorld;
                const Vec3 innerAt = physical.feet[inner].supporting ? physical.feet[inner].soleCenterWorld
                    : plan.feet[inner].anchorWorld;
                const float beyond = dot(Vec3 { input.capturePointWorld.x - outerAt.x,
                    input.capturePointWorld.y - outerAt.y, 0.0f }, direction);
                const float width = dot(Vec3 { outerAt.x - innerAt.x, outerAt.y - innerAt.y, 0.0f }, direction);
                int stepping = -1;
                const bool fallingOff = outsideBase > RecoveryOutsideTrigger
                    && beyond <= BrakeStepTrigger && !behind;
                // (O ponto de captura ja alem do pe de fora: ele abre antes de
                // o de dentro fechar - fechando primeiro com o corpo a 1,5 m/s
                // de lado, o corpo passava do pe de fora e caia.)
                if (SideOpenFirst && beyond > SideOpenFirstBeyond) {
                    stepping = static_cast<int>(outer);
                    m_brakeClosing[outer] = false;
                } else if (width > BrakeSideClose && !fallingOff) {
                    stepping = static_cast<int>(inner);
                    m_brakeClosing[inner] = true;
                } else if (beyond > BrakeStepTrigger) {
                    stepping = static_cast<int>(outer);
                    m_brakeClosing[outer] = false;
                } else if (behind) {
                    stepping = static_cast<int>(inner);
                    m_brakeClosing[inner] = false;
                }
                plan.brakeYield = stepping < 0 && outsideBase > RecoveryOutsideTrigger;
                if (stepping >= 0) {
                    const std::size_t side = static_cast<std::size_t>(stepping);
                    m_brakeRearLanding[side] = false;
                    ++m_brakeStepsTaken;
                    startStep(profile, input, side, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
                    plan.feet[side].swingDurationSeconds = BrakeSwingSeconds;
                }
            } else if (captureAhead > BrakeStepTrigger || behind
                || (closing && outsideBase <= RecoveryOutsideTrigger)) {
                m_brakeClosing[trailing] = false;
                m_brakeRearLanding[trailing] = false;
                ++m_brakeStepsTaken;
                startStep(profile, input, trailing, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
                plan.feet[trailing].swingDurationSeconds = BrakeSwingSeconds;
            } else {
                plan.brakeYield = outsideBase > RecoveryOutsideTrigger;
            }
        }
    }
    // Segundo passo: o primeiro pousou (apoia) e o outro pe sai na hora - a
    // passada assume com ele no ar (o ciclo de corrida nao tem apoio duplo:
    // devolvendo com os dois no chao, um deles era arrastado).
    if (planning && input.stumbling && !input.braking && m_stumbleStepsTaken == 1) {
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t other = 1 - side;
            if (!m_stumbleStep[other] || m_stumbleStep[side]) continue;
            const FootPlan3D& first = plan.feet[other];
            if (first.phase != FootPhase3D::Stance && first.phase != FootPhase3D::Loading) continue;
            if (plan.feet[side].phase != FootPhase3D::Stance) continue;
            m_stumbleStep[side] = true;
            m_brakeClosing[side] = false;
            m_brakeRearLanding[side] = false;
            ++m_stumbleStepsTaken;
            startStep(profile, input, side, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
            plan.feet[side].swingDurationSeconds = input.stumbleSwingSeconds;
            break;
        }
    }
    // Quase-queda com os dois pes no chao na troca: sai o de tras (no rumo).
    if (planning && input.stumbling && !input.braking && m_stumbleStepsTaken == 0
        && m_stumbleSeconds >= input.stumbleStepDelaySeconds) {
        bool busy = false;
        for (const FootPlan3D& foot : plan.feet)
            busy = busy || airborne(foot.phase) || foot.phase == FootPhase3D::Unloading;
        Vec3 direction = input.stumbleDirectionWorld;
        direction.z = 0.0f;
        if (!busy && direction.lengthSquared() > 1e-4f) {
            direction = direction.normalized();
            const std::size_t trailing = dot(plan.feet[0].anchorWorld, direction)
                < dot(plan.feet[1].anchorWorld, direction) ? 0 : 1;
            m_stumbleStep[trailing] = true;
            m_brakeClosing[trailing] = false;
            m_brakeRearLanding[trailing] = false;
            ++m_stumbleStepsTaken;
            startStep(profile, input, trailing, StepReason3D::Brake, SwingPath3D::Direct, 1.0f);
            plan.feet[trailing].swingDurationSeconds = input.stumbleSwingSeconds;
        }
    }
    if (std::getenv("XTURNDBG") && input.standing)
        std::printf("TURNDBG planning %d standing %d steps %d gait %d%d | fases %d %d | cooldown %.2f %.2f | recovering %d lado %d urg %.2f | pending %d | alvo %+.1f rumo %+.1f\n",
            planning ? 1 : 0, input.standing ? 1 : 0, input.allowSteps ? 1 : 0, plan.feet[0].followingGait ? 1 : 0,
            plan.feet[1].followingGait ? 1 : 0, static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase),
            m_stepCooldown[0], m_stepCooldown[1], plan.recovering ? 1 : 0, plan.recoverySide, plan.recoveryUrgency,
            m_pendingTurnFoot, input.standingTurnTargetYawRadians * 57.2958f, input.facingYawRadians * 57.2958f);
    if (planning && !locomotionPlanning && !input.braking && !input.stumbling && m_settings.benchTransferFoot < 0) {
        const bool bothStance = plan.feet[0].phase == FootPhase3D::Stance
            && plan.feet[1].phase == FootPhase3D::Stance;
        // Um passo de acomodacao ou de giro em andamento vira de recuperacao
        // (o pe muda de alvo no ar); com os dois apoiados, sai o passo.
        if (plan.recoverySide >= 0 && plan.recoveryUrgency > 0.25f) {
            for (std::size_t side = 0; side < 2; ++side) {
                const FootPlan3D& foot = plan.feet[side];
                // (So o pe escolhido: caindo de lado, o de dentro no ar virava
                // recuperacao e, sem poder cruzar, mirava para FORA da queda -
                // parando de um strafe a esquerda a 5%, 14 quedas em 16. Ele
                // termina o pouso dele e o de fora abre em seguida.)
                if (RecoverOnlyChosenFoot && static_cast<int>(side) != plan.recoverySide
                    && foot.reason != StepReason3D::Settle && foot.reason != StepReason3D::Turn) continue;
                // (O pe roteirizado - toque parado na bola - segue o roteiro.)
                if (input.scripted[side].active) continue;
                if (foot.phase == FootPhase3D::Unloading || airborne(foot.phase)) {
                    makeRecovery(input, side, plan.recoveryUrgency);
                    m_pendingTurnFoot = -1;
                }
            }
        }
        const Vec3 headingForward = input.heading.rotate({ 1.0f, 0.0f, 0.0f });
        // Base escalonada (um pe bem a frente do outro, como a base em
        // alerta) ou lado a lado.
        const float leftAhead = dot(input.reference[0].poseWorld
            - input.reference[1].poseWorld, headingForward);
        const bool staggered = std::abs(leftAhead) > 0.15f;
        // Sem a ajuda (captura), com o corpo devagar o passo de recuperacao
        // vai para a pose; se o pe ja esta nela (5 cm), o passo nao muda
        // nada - so tira o pe do chao e balanca o corpo (parado, passos de
        // 2-4 cm um atras do outro). O equilibrio pelo centro de pressao
        // cuida. (Com forca de fora - uma caixa empurrando devagar - o passo
        // sai: pulado ali, a caixa derrubava.)
        bool pointless = false;
        if (plan.recoverySide >= 0 && m_settings.recoveryCaptureTargeting) {
            const std::size_t side = static_cast<std::size_t>(plan.recoverySide);
            const float speed = std::hypot(input.planningComVelocityWorld.x,
                input.planningComVelocityWorld.y);
            const Vec3 pose = input.reference[side].poseWorld;
            pointless = speed < CaptureTargetSlowSpeed
                && physical.externalForceWorld.length() < RecoveryForceTrigger
                && std::hypot(pose.x - plan.feet[side].anchorWorld.x,
                    pose.y - plan.feet[side].anchorWorld.y) < 0.05f;
        }
        // Um giro em andamento (o segundo pe ainda por girar) segue com uma
        // recuperacao leve: cancelado, o giro recomecava pelo mesmo pe e a
        // pelve ficava presa no limite de torcao do outro.
        const bool turnPending = m_pendingTurnFoot >= 0 && plan.recoveryUrgency < TurnRecoveryUrgency;
        if (plan.recoverySide >= 0 && bothStance && !pointless && !turnPending) {
            m_pendingTurnFoot = -1;
            startStep(profile, input, static_cast<std::size_t>(plan.recoverySide),
                StepReason3D::Recovery, SwingPath3D::Direct, plan.recoveryUrgency);
        } else if (bothStance && (!input.settleHold || UnreachableDuringSettleHold)
            && (!plan.recovering || turnPending || (m_turnRequested && plan.recoverySide < 0))) {
            int unreachable = -1;
            // (Recem-freado, so o pe que nao apoia de fato: o que ficou longe,
            // pendurado, nao segura nada - esperando a acomodacao, ele ficava
            // 0,8 m a frente do corpo, no ar, por 1,5 s.)
            for (std::size_t side = 0; side < 2; ++side)
                if (input.anchorUnreachable[side] && ready(side)
                    && (!input.settleHold || !contactSupport(physical.feet[side])))
                    unreachable = static_cast<int>(side);
            // Ajuda cheia: o segundo passo do giro vem antes (a acomodacao do pe
            // que acabou de girar, "fora do alcance" com o corpo ainda girando,
            // cancelava o giro: 90 graus levavam 0,85 s em vez de 0,26).
            // (Com qualquer ajuda: o pe fora de alcance, logo depois do
            // primeiro passo do giro, dava um passo de acomodacao que quebrava
            // o par e o giro recomecava 0,8 s depois.)
            if (m_pendingTurnFoot >= 0) unreachable = -1;
            if (unreachable >= 0) {
                // A perna nao alcanca mais o pe apoiado (o corpo se afastou):
                // ele da o passo ate a base.
                startStep(profile, input, static_cast<std::size_t>(unreachable),
                    StepReason3D::Settle, SwingPath3D::AroundSupport, 0.0f);
                m_pendingTurnFoot = -1;
            } else if (input.settleHold) {
                // Recem-freado: nenhuma acomodacao nem giro ainda - salvo a
                // base cruzada ou em linha (de lado a freada termina as vezes
                // com um pe na frente do outro, 4 cm cruzados: ficava assim
                // 1,5 s, feio e sem base de lado). Sai o pe mais longe da pose.
                const Vec3 leftAxis = input.heading.rotate({ 0.0f, 1.0f, 0.0f });
                const float width = dot(plan.feet[0].anchorWorld - plan.feet[1].anchorWorld, leftAxis);
                if (SettleHoldNarrowBase > 0.0f && width < SettleHoldNarrowBase && ready(0) && ready(1)) {
                    int worst = -1;
                    float worstDrift = 0.0f;
                    for (std::size_t side = 0; side < 2; ++side) {
                        const Vec3 pose = input.reference[side].poseWorld;
                        const float drift = std::hypot(pose.x - plan.feet[side].anchorWorld.x,
                            pose.y - plan.feet[side].anchorWorld.y);
                        if (drift > worstDrift) { worstDrift = drift; worst = static_cast<int>(side); }
                    }
                    if (worst >= 0 && worstDrift > 0.06f)
                        startStep(profile, input, static_cast<std::size_t>(worst),
                            StepReason3D::Settle, SwingPath3D::AroundSupport, 0.0f);
                }
            } else if (m_pendingTurnFoot >= 0) {
                // Segundo passo do giro, logo depois do primeiro: se o pe
                // ainda esta fora da base nova, contorna o de apoio ate ela
                // (desistido o primeiro por falta de carga, espera).
                if (ready(static_cast<std::size_t>(m_pendingTurnFoot))) {
                    const std::size_t side = static_cast<std::size_t>(m_pendingTurnFoot);
                    m_pendingTurnFoot = -1;
                    FootPlan3D& foot = plan.feet[side];
                    const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
                    const StepReason3D previousReason = foot.reason;
                    const float previousFacing = foot.turnFromFacing;
                    foot.reason = StepReason3D::Turn;
                    foot.turnFromFacing = input.facingYawRadians;
                    m_turnSign[side] = wrapAngle(input.standingTurnTargetYawRadians
                        - input.facingYawRadians) >= 0.0f ? 1.0f : -1.0f;
                    Quaternion orientation;
                    const Vec3 position = wantedLanding(input, side, orientation);
                    foot.reason = previousReason;
                    foot.turnFromFacing = previousFacing;
                    const float drift = std::hypot(position.x - foot.anchorWorld.x,
                        position.y - foot.anchorWorld.y);
                    const float yaw = std::abs(wrapAngle(footYaw3D(link, orientation)
                        - footYaw3D(link, foot.anchorOrientationWorld)));
                    if (drift > 0.05f || yaw > 0.17f)
                        startStep(profile, input, side, StepReason3D::Turn, SwingPath3D::AroundSupport, 0.0f);
                }
            } else {
                const float stance = stanceYaw(profile, input);
                const float twist = wrapAngle(input.facingYawRadians - stance);
                const float remaining = wrapAngle(input.standingTurnTargetYawRadians - stance);
                plan.standingStanceYaw = stance;
                plan.standingTwist = twist;
                plan.standingTurnRemaining = remaining;
                // Ajuda cheia segurando, sem a camera girar de verdade: a base
                // torta (a freada termina com o ultimo pe da passada reto e a
                // pose parada e rodada ~21 graus) se acerta com um passo de
                // acomodacao do pe mais fora da pose - eram dois passos de
                // giro 1,5 s depois de parar.
                // (So logo depois de uma freada e sem giro pedido - a torcao
                // e do tipo de base, nao do olhar. Com o giro pedido, o giro
                // normal: segurado, o corpo girava com os pes presos e as
                // pernas torciam.)
                const bool heldStance = input.assistHold > 0.5f && input.quietStop
                    && std::abs(input.standingTurnRateRadiansPerSecond) < 0.3f
                    && std::abs(wrapAngle(input.standingTurnTargetYawRadians - input.facingYawRadians)) < 0.1f
                    && std::abs(twist) < HeldTurnTwist && std::abs(remaining) < HeldTurnTwist;
                const bool turnNeeded = std::abs(twist) > TurnTwistTrigger || std::abs(remaining) > TurnEarlyRemaining;
                if (!heldStance && turnNeeded) {
                    // Base lado a lado: o pe do lado do giro primeiro, abrindo
                    // sobre a bola do pe. Base escalonada: para qualquer lado,
                    // o pe de TRAS primeiro, contornando o da frente com o
                    // peso nele; depois o da frente refaz a base.
                    // (So "o de tras primeiro": no modo fisico, girando para o
                    // lado do pe da frente, a recuperacao cancelava o giro no
                    // meio e ele recomecava pelo mesmo pe - o outro prendia a
                    // pelve no limite de torcao e ele nunca girava para aquele
                    // lado.)
                    // Sai primeiro o pe mais atrasado no giro (o rumo dele
                    // contra o alvo): depois de um giro parcial, o que ja
                    // girou saia de novo e o outro, que prendia a pelve no
                    // limite de torcao, nunca girava. Empatados (10 graus -
                    // o comeco do giro): na base escalonada o de tras (ele
                    // contorna o da frente), lado a lado o do lado do giro.
                    std::array<float, 2> behind {};
                    for (std::size_t side = 0; side < 2; ++side) {
                        const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
                        const float open = wrapAngle(footYaw3D(link, input.reference[side].poseOrientationWorld)
                            - input.facingYawRadians);
                        behind[side] = std::abs(wrapAngle(input.standingTurnTargetYawRadians
                            - (footYaw3D(link, plan.feet[side].anchorOrientationWorld) - open)));
                    }
                    // (Com o equilibrio pelas pernas - captura, E4/E5 -, o de
                    // tras contornando o da frente era lento, 0,7 s, e o
                    // corpo num pe so saia 30 cm da base: la sempre o do lado
                    // do giro, abrindo sobre a bola do pe.)
                    const bool rearFirst = staggered && !m_settings.recoveryCaptureTargeting;
                    std::size_t first = std::abs(behind[0] - behind[1]) > 0.17f
                        ? (behind[0] > behind[1] ? 0 : 1)
                        : rearFirst ? (leftAhead > 0.0f ? 1 : 0) : (remaining > 0.0f ? 0 : 1);
                    bool rearAround = rearFirst && first == (leftAhead > 0.0f ? 1u : 0u);
                    if (ready(first)) {
                        startStep(profile, input, first, StepReason3D::Turn,
                            rearAround ? SwingPath3D::AroundSupport : SwingPath3D::Pivot, 0.0f);
                        m_pendingTurnFoot = static_cast<int>(1 - first);
                    }
                } else {
                    int worst = -1;
                    float worstScore = 0.0f;
                    // Ajuda cheia: a base e a geometria entre os pes (o corpo
                    // se assenta sobre ela): cada pe contra onde a pose parada o
                    // quer em relacao ao outro, e quem sai e o mais longe do
                    // corpo (o de menos peso). Contra a pose presa a raiz, o pe
                    // da freada nunca estava "no lugar" e saiam 2-3 passos.
                    const bool pairStance = input.assistHold > 0.5f;
                    // (Girando, quem acerta a base sao os passos de giro; ao
                    // terminar, uma acomodacao devolve a base parada inteira.)
                    if (pairStance && !m_turnMode) {
                        std::array<float, 2> score {};
                        for (std::size_t side = 0; side < 2; ++side) {
                            const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
                            const Vec3 target = pairStanceTarget(input, side);
                            const float drift = std::hypot(target.x - plan.feet[side].anchorWorld.x,
                                target.y - plan.feet[side].anchorWorld.y);
                            const float yaw = std::abs(wrapAngle(footYaw3D(link, input.reference[side].poseOrientationWorld)
                                - footYaw3D(link, plan.feet[side].anchorOrientationWorld)));
                            // (Logo depois de um giro, a volta para a base inteira
                            // sai com menos: a base encurtada do giro fica ~15 cm
                            // fora.)
                            const float distanceLimit = m_turnModeSeconds < 2.0f
                                && (TurnStanceScale < 0.999f || TurnStanceHalfWidth > 0.0f)
                                ? SettleStepDistance * 0.5f : SettleStepDistance;
                            score[side] = std::max(drift / distanceLimit, yaw / SettleStepYaw);
                        }
                        if (std::getenv("XSETTLEDBG") && std::max(score[0], score[1]) > 1.0f)
                            for (std::size_t side = 0; side < 2; ++side) {
                                const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
                                const Vec3 target = pairStanceTarget(input, side);
                                std::printf("SETTLE pe %zu score %.2f | alvo (%.2f %.2f) ancora (%.2f %.2f) | yaw ref %.1f ancora %.1f\n",
                                    side, score[side], target.x, target.y, plan.feet[side].anchorWorld.x, plan.feet[side].anchorWorld.y,
                                    footYaw3D(link, input.reference[side].poseOrientationWorld) * 57.2958f,
                                    footYaw3D(link, plan.feet[side].anchorOrientationWorld) * 57.2958f);
                            }
                        // Uma acomodacao por parada: a pose parada em alerta abre o
                        // pe de tras ~0,6 m e 46 graus; com o corpo ainda se
                        // assentando a perna nao chegava (~19 cm curta) e o passo
                        // se repetia a cada 0,27 s no mesmo lugar.
                        if (std::max(score[0], score[1]) > 1.0f && m_heldSettles < 1) {
                            ++m_heldSettles;
                            const auto load = [&](std::size_t side) {
                                return -std::hypot(plan.feet[side].anchorWorld.x - physical.centerOfMassWorld.x,
                                    plan.feet[side].anchorWorld.y - physical.centerOfMassWorld.y);
                            };
                            const std::size_t mover = load(0) < load(1) ? 0 : 1;
                            if (ready(mover))
                                startStep(profile, input, mover, StepReason3D::Settle, SwingPath3D::AroundSupport, 0.0f);
                        }
                    }
                    for (std::size_t side = 0; side < 2 && !pairStance; ++side) {
                        const FootPlan3D& foot = plan.feet[side];
                        const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
                        const FootReference3D& wanted = input.reference[side];
                        const float drift = std::hypot(wanted.poseWorld.x - foot.anchorWorld.x,
                            wanted.poseWorld.y - foot.anchorWorld.y);
                        const float yaw = std::abs(wrapAngle(footYaw3D(link, wanted.poseOrientationWorld)
                            - footYaw3D(link, foot.anchorOrientationWorld)));
                        const float score = std::max(drift / SettleStepDistance, yaw / SettleStepYaw);
                        if (score > 1.0f && score > worstScore && ready(side)) {
                            worstScore = score;
                            worst = static_cast<int>(side);
                        }
                    }
                    if (worst >= 0) {
                        startStep(profile, input, static_cast<std::size_t>(worst),
                            StepReason3D::Settle, SwingPath3D::AroundSupport, 0.0f);
                    }
                }
            }
        }
    }

    // Execucao de cada pe do planejador.
    const bool staggeredBase = std::abs(dot(input.reference[0].poseWorld
        - input.reference[1].poseWorld, input.heading.rotate({ 1.0f, 0.0f, 0.0f }))) > 0.15f;
    for (std::size_t side = 0; side < 2; ++side) {
        FootPlan3D& foot = plan.feet[side];
        if (foot.followingGait) continue;
        // Apoio planejado contra observado (R1). O apoio ficticio e contado;
        // quem aloca esforco de contato (AdaptivePhysicalCharacter3D) usa o
        // observado.
        {
            const FootSupportEstimate3D& seen = physical.feet[side];
            const Vec3 slip = seen.soleVelocityWorld;
            foot.observedSupport = !seen.contactObserved
                ? (seen.touching ? ObservedSupport3D::Touching : ObservedSupport3D::None)
                : std::hypot(slip.x, slip.y) >= LoadAcceptanceSlipSpeed ? ObservedSupport3D::Slipping
                : ObservedSupport3D::Supporting;
            foot.plannedSupport = foot.phase == FootPhase3D::Stance ? PlannedSupport3D::Stance
                : foot.phase == FootPhase3D::Loading ? PlannedSupport3D::Accepting
                : foot.phase == FootPhase3D::Unloading ? PlannedSupport3D::TransferOut
                : PlannedSupport3D::None;
            const bool lost = foot.plannedSupport != PlannedSupport3D::None
                && (foot.observedSupport == ObservedSupport3D::None
                    || foot.observedSupport == ObservedSupport3D::Touching);
            if (lost) ++m_plan.fakeSupportTicks[static_cast<std::size_t>(foot.plannedSupport) - 1];
            if (lost && !m_supportLost[side]) ++m_plan.supportLostEvents;
            m_supportLost[side] = lost;
        }
        // Freando, o pe apoiado que derrapa leva a ancora junto: o corpo freia
        // pelo atrito com a perna firme. (Parada no mundo com o pe
        // escorregando, a perna puxava o pe de volta e lancava o corpo.)
        if (input.braking && supportsWeight(foot.phase)) {
            const FootSupportEstimate3D& seen = physical.feet[side];
            const float slide = std::hypot(seen.soleVelocityWorld.x, seen.soleVelocityWorld.y);
            if (seen.contactObserved && slide > BrakeSlideSpeed) {
                const Vec3 at = input.state->links[m_feet[side]].position;
                foot.anchorWorld.x = at.x;
                foot.anchorWorld.y = at.y;
            }
        }
        switch (foot.phase) {
        case FootPhase3D::Stance:
            foot.targetWorld = foot.anchorWorld;
            foot.targetOrientationWorld = foot.anchorOrientationWorld;
            break;
        case FootPhase3D::Unloading: {
            foot.phaseSeconds += deltaTime;
            foot.targetWorld = foot.anchorWorld;
            foot.targetOrientationWorld = foot.anchorOrientationWorld;
            // Sai quando a carga passou para o outro pe (ou quase, depois de
            // tentar). Na recuperacao sai logo de qualquer jeito (quanto mais
            // urgente, menos espera: liberar um pe cedo vale mais que uma
            // troca de peso perfeita).
            const bool recovery = foot.reason == StepReason3D::Recovery
                || foot.reason == StepReason3D::Brake;
            const bool otherOnGround = supportsWeight(plan.feet[1 - side].phase);
            const bool byLoad = otherOnGround && plan.loadShare[side] < UnloadedShare;
            const bool byPartialLoad = otherOnGround && foot.phaseSeconds >= PartialUnloadSeconds
                && plan.loadShare[side] < PartialUnloadShare;
            bool byGeometry = false, byAirborne = false;
            // Marcha E5: o pe de tras sai quando o da frente ja sustenta o apoio
            // simples - o ponto de captura (DCM) fez 60% do caminho de uma sola
            // a outra, com o pe da frente apoiado de fato. Pela carga nao da:
            // com o corpo acelerando, o centro de pressao fica no pe de tras
            // (ele empurra), e 34 de 39 passos saiam carregados pelo prazo.
            if (m_settings.locomotionStepping && m_settings.transferGeometryRelease
                && foot.reason == StepReason3D::Locomotion && otherOnGround
                && physical.feet[1 - side].supporting) {
                const Vec3 from = physical.feet[side].soleCenterWorld;
                Vec3 axis = physical.feet[1 - side].soleCenterWorld - from;
                axis.z = 0.0f;
                const float length = axis.lengthSquared();
                const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
                Vec3 dcm = physical.centerOfMassWorld + physical.centerOfMassVelocityWorld * (1.0f / omega) - from;
                dcm.z = 0.0f;
                const float progress = length > 1e-4f ? dot(dcm, axis) / length : 0.0f;
                byGeometry = progress >= LocomotionTransferDone;
            }
            // Com o pouso previsto (fase 11): o pe que sai a frente do outro no
            // rumo da marcha (de lado, o que abre) sai pela agenda - o outro
            // aceitou o apoio e passou a dupla sustentacao minima -, como o
            // modelo do pouso supoe. Ele nao descarrega esperando: o corpo vai
            // para ele e o DCM nunca vai ao de tras; segurava a carga ate o
            // prazo com o corpo passando dele.
            bool leadingRelease = false;
            if (m_settings.locomotionStepping && m_settings.locomotionPlacementPredictor
                && foot.reason == StepReason3D::Locomotion && otherOnGround
                && physical.feet[1 - side].supporting && foot.phaseSeconds >= LocomotionDoubleSupportSeconds) {
                Vec3 travel = input.desiredVelocityWorld;
                travel.z = 0.0f;
                const float travelSpeed = travel.length();
                const Vec3 ahead = physical.feet[side].soleCenterWorld - physical.feet[1 - side].soleCenterWorld;
                leadingRelease = travelSpeed > LocomotionMinimumSpeed
                    && dot(ahead, travel) / travelSpeed > 0.05f;
            }
            // (Pacote do GPT, D6: o solver decide o contato.) O pe da marcha
            // que ja saiu do chao - sem apoio medido e com a sola acima do
            // chao dele - esta descarregado: sai agora. Esperando a carga
            // estimada cair, ele balancava solto "em apoio" ate 50 ms e saia
            // 4-9 cm fora da ancora.
            if (m_settings.locomotionStepping && foot.reason == StepReason3D::Locomotion && otherOnGround
                && physical.feet[1 - side].supporting && !physical.feet[side].supporting
                && physical.feet[side].soleLowestHeight > foot.surface.height + 0.01f)
                byAirborne = true;
            const bool unloaded = byLoad || byPartialLoad || byGeometry || byAirborne;
            // A razao registrada (R2), da mais forte para a mais fraca.
            const ReleaseReason3D reason = recovery ? ReleaseReason3D::Recovery
                : byAirborne ? ReleaseReason3D::AirborneObserved
                : byLoad ? ReleaseReason3D::LoadEstimate
                : byGeometry ? ReleaseReason3D::TransferGeometry
                : leadingRelease ? ReleaseReason3D::LeadingAgenda
                : byPartialLoad ? ReleaseReason3D::PartialLoadEstimate
                : ReleaseReason3D::TransferFailed;
            // No passo comum, em no maximo 0,3 s: se a carga nao passou, o pe
            // sai carregado e e arrastado (medido e contado; a troca de peso
            // de verdade e da atuacao das juntas, E4) - desistir deixava a
            // base torta e o corpo balancando a cada nova tentativa.
            // Sem a ajuda (captura) a descarga e das pernas: um empurrao forte
            // solta logo; um desequilibrio lento (arrastado pela PhysGun)
            // espera ate 0,15 s o peso passar - solto carregado, o pe so
            // arrastava no chao e o corpo tombava.
            const float waitLimit = foot.reason == StepReason3D::Brake ? 0.0f
                : recovery
                ? (m_settings.recoveryCaptureTargeting ? RecoveryCaptureUnloadSeconds * (1.0f - foot.urgency)
                    : RecoveryUnloadMaximumSeconds * (1.0f - foot.urgency))
                : foot.reason == StepReason3D::Locomotion ? LocomotionUnloadSeconds
                : foot.reason == StepReason3D::Turn
                    ? TurnUnloadSeconds * (input.assistHold > 0.5f ? HeldTurnUnloadShare : 1.0f)
                    : UnloadMaximumSeconds;
            // Bancada C: descarrega sem soltar; desligada, o pe volta a apoiar.
            if (m_settings.benchTransferFoot >= 0 || m_benchHolding[side]) {
                if (m_settings.benchTransferFoot != static_cast<int>(side)) {
                    foot.phase = FootPhase3D::Stance;
                    foot.reason = StepReason3D::None;
                    foot.phaseSeconds = 0.0f;
                    m_benchHolding[side] = false;
                }
                break;
            }
            if (unloaded || leadingRelease || foot.phaseSeconds >= waitLimit) {
                if (foot.reason == StepReason3D::Locomotion && plan.loadShare[side] >= UnloadedShare) {
                    if (unloaded) ++m_plan.locomotionPartialReleases;
                    else if (leadingRelease) ++m_plan.locomotionLeadingReleases;
                    else ++m_plan.locomotionTimeoutReleases;
                }
                float fraction = 0.0f, limit = 0.0f;
                unloadShift(foot.phaseSeconds, staggeredBase, fraction, limit);
                release(profile, input, side, reason);
                m_releaseShiftFraction[side] = fraction;
                m_releaseShiftLimit[side] = limit;
            }
            break;
        }
        case FootPhase3D::Swing:
        case FootPhase3D::TouchdownSearch:
            executeSwing(profile, input, side, deltaTime);
            break;
        case FootPhase3D::Loading:
            foot.phaseSeconds += deltaTime;
            foot.targetWorld = foot.anchorWorld;
            foot.targetOrientationWorld = foot.anchorOrientationWorld;
            if (m_settings.locomotionStepping && m_settings.loadAcceptanceByEvent) {
                // E5 (fase 5): a carga termina quando o pe recebe o apoio de
                // fato - apoiado sem interrupcao por 25 ms e sem escorregar -,
                // nao pelo tempo. Passado o prazo, conta e continua carregando;
                // no teto (0,4 s) vira apoio com falha explicita. (Por 50 ms, o
                // pe de tras que fechava o passo lateral quicava fora do chao
                // e o da frente ja comecava a descarregar, a toa, ate o prazo.)
                const FootSupportEstimate3D& measured = physical.feet[side];
                const Vec3 slip = measured.soleVelocityWorld;
                const bool accepted = measured.supporting
                    && measured.supportingSeconds >= LoadAcceptanceSeconds
                    && std::hypot(slip.x, slip.y) < LoadAcceptanceSlipSpeed;
                if (!accepted && foot.phaseSeconds >= LoadingMaximumSeconds
                    && foot.phaseSeconds - deltaTime < LoadingMaximumSeconds)
                    ++m_plan.loadingTimeouts;
                if (accepted || foot.phaseSeconds >= LoadAcceptanceLimitSeconds) {
                    if (!accepted) ++m_plan.loadAcceptanceFailures;
                    foot.phase = FootPhase3D::Stance;
                    foot.phaseSeconds = 0.0f;
                }
                break;
            }
            if (plan.loadShare[side] > LoadedShare || foot.phaseSeconds >= LoadingMaximumSeconds) {
                if (plan.loadShare[side] <= LoadedShare) ++m_plan.loadingTimeouts;
                foot.phase = FootPhase3D::Stance;
                foot.phaseSeconds = 0.0f;
            }
            break;
        }
    }

    // Transferencia de apoio (fase 4): um pe descarregando para um passo, o
    // outro apoiado. O centro de massa vai ate 85% do caminho ao apoio do
    // outro, numa rampa pelo motivo (recuperacao 0,08 s, marcha 0,12, o resto
    // 0,20). Antes esse calculo vivia no controle das pernas.
    plan.supportTransfer = {};
    for (std::size_t side = 0; side < 2; ++side) {
        const FootPlan3D& foot = plan.feet[side];
        if (foot.followingGait || foot.phase != FootPhase3D::Unloading) continue;
        if (!supportsWeight(plan.feet[1 - side].phase)) continue;
        const float ramp = foot.reason == StepReason3D::Recovery
            || foot.reason == StepReason3D::Brake ? 0.08f
            : foot.reason == StepReason3D::Locomotion ? 0.12f : 0.20f;
        plan.supportTransfer.outgoingFoot = static_cast<int>(side);
        plan.supportTransfer.incomingFoot = static_cast<int>(1 - side);
        plan.supportTransfer.progress = 0.85f * std::clamp(foot.phaseSeconds / ramp, 0.0f, 1.0f);
        plan.supportTransfer.supportPointWorld = physical.feet[1 - side].soleCenterWorld;
    }

    // Peso para o pe de apoio num passo comum (acomodacao, giro, refazer a
    // base): a pelve vai na direcao dele e baixa um pouco na descarga (ver
    // unloadShift), fica assim ate a metade do balanco e volta ate o pouso.
    // Na recuperacao, nao.
    Vec3 requested {};
    for (std::size_t side = 0; side < 2; ++side) {
        const FootPlan3D& foot = plan.feet[side];
        const FootPlan3D& support = plan.feet[1 - side];
        // Pe roteirizado com peso no apoio (toque parado na bola): a pelve vai
        // a fracao pedida do caminho ate o outro pe, enquanto ele apoia.
        const ScriptedFoot3D& script = rawInput.scripted[side];
        if (script.active && script.supportShift > 0.0f && supportsWeight(support.phase)) {
            const Vec3 lean { support.anchorWorld.x - input.referenceRootWorld.x,
                support.anchorWorld.y - input.referenceRootWorld.y, 0.0f };
            Vec3 shift = clampMagnitude(lean * std::min(script.supportShift, 1.0f), ScriptedShiftLimit);
            shift.z = -0.015f;
            requested = shift;
            continue;
        }
        if (foot.followingGait || foot.reason == StepReason3D::Recovery
            || foot.reason == StepReason3D::Brake
            || support.phase != FootPhase3D::Stance) continue;
        float fraction = 0.0f, limit = 0.0f;
        if (foot.phase == FootPhase3D::Unloading) {
            unloadShift(foot.phaseSeconds, staggeredBase, fraction, limit);
        } else if (airborne(foot.phase)) {
            const float progress = std::clamp(foot.swingSeconds
                / std::max(0.05f, foot.swingDurationSeconds), 0.0f, 1.0f);
            const float hold = progress < 0.5f ? 1.0f : std::sin(Pi * progress);
            fraction = m_releaseShiftFraction[side] * hold;
            limit = m_releaseShiftLimit[side] * hold;
        }
        if (fraction <= 0.0f) continue;
        const Vec3 lean { support.anchorWorld.x - input.referenceRootWorld.x,
            support.anchorWorld.y - input.referenceRootWorld.y, 0.0f };
        Vec3 shift = clampMagnitude(lean * fraction, limit);
        shift.z = -0.015f * std::min(1.0f, fraction / 0.25f);
        requested = shift;
    }
    // O deslocamento pedido muda de lado ou some num tick (um passo desiste e
    // o do outro pe comeca, pousou, virou recuperacao); a pelve vai atras em
    // ~50 ms, nunca num tick (medido: 1,5 cm de pulo da pelve).
    m_residualShift += (requested - m_residualShift) * (1.0f - std::exp(-deltaTime / 0.05f));
    plan.supportShiftWorld = m_residualShift;

    // Pes roteirizados (giro parado pelo clipe): no ar o pe segue o do clipe;
    // pousando, trava no plano onde o clipe o pos, na altura da trava antiga
    // e so com o rumo do clipe (sola plana).
    for (std::size_t side = 0; side < 2; ++side) {
        const ScriptedFoot3D& script = rawInput.scripted[side];
        FootPlan3D& foot = plan.feet[side];
        if (!script.active || m_feet[side] >= profile.links.size()) {
            m_scriptedLast[side] = false;
            continue;
        }
        const RagdollLinkDefinition3D& link = profile.links[m_feet[side]];
        const bool down = supportsWeight(foot.phase);
        foot.followingGait = false;
        if (script.swing) {
            if (down || !m_scriptedLast[side] || foot.phase != FootPhase3D::Swing) {
                foot.liftoffWorld = foot.anchorWorld;
                foot.liftoffOrientationWorld = foot.anchorOrientationWorld;
                foot.liftoffGroundHeight = foot.surface.valid ? foot.surface.height : foot.anchorWorld.z;
                foot.stepId = m_nextStepId++;
                foot.reason = StepReason3D::Turn;
                foot.path = SwingPath3D::Direct;
                foot.phaseSeconds = 0.0f;
                foot.swingSeconds = 0.0f;
                ++m_plan.stepsStarted;
            }
            foot.phase = FootPhase3D::Swing;
            foot.swingSeconds += deltaTime;
            foot.targetWorld = script.positionWorld;
            foot.targetOrientationWorld = script.orientationWorld;
            foot.goalWorld = script.positionWorld;
            foot.goalOrientationWorld = script.orientationWorld;
            foot.goalSurface = foot.surface;
        } else if (!down) {
            // O clipe pousou o pe: ele vai ao chao no ponto do clipe e procura
            // o contato; so tocando de fato entra em carga (o contrato do pe)
            // e depois em apoio. No tempo acelerado o pe fisico chega uns
            // centimetros depois do clipe.
            // Procurando o chao, o alvo desce aos poucos (ate 4 cm): no toque
            // parado na bola o pe ficava 1-2 cm acima, sem contato, ate o prazo.
            const float search = foot.phase == FootPhase3D::TouchdownSearch
                ? std::min(0.04f, foot.phaseSeconds * 0.25f) : 0.0f;
            // Na altura de onde saiu ou na dada, a mais baixa (o pe de tras da
            // base em alerta sai com o calcanhar alto: pousando nessa altura,
            // plano, ficava 2 cm acima do chao).
            const Vec3 landing { script.positionWorld.x, script.positionWorld.y,
                std::min(foot.liftoffWorld.z, script.positionWorld.z) - search };
            const Quaternion flat = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                wrapAngle(footYaw3D(link, script.orientationWorld) - footYaw3D(link, foot.liftoffOrientationWorld)))
                * foot.liftoffOrientationWorld).normalized();
            if (foot.phase != FootPhase3D::TouchdownSearch) foot.phaseSeconds = 0.0f;
            const bool touching = physical.feet[side].touching || physical.feet[side].supporting;
            if (std::getenv("XSCRIPTDBG"))
                std::printf("SCRIPT lado %zu fase %d t %.3f toca %d alvo z %.3f liftoff z %.3f\n", side,
                    static_cast<int>(foot.phase), foot.phaseSeconds, touching ? 1 : 0, landing.z, foot.liftoffWorld.z);
            if (touching || (foot.phase == FootPhase3D::TouchdownSearch && foot.phaseSeconds > 0.2f)) {
                foot.anchorWorld = landing;
                foot.anchorOrientationWorld = flat;
                foot.phase = FootPhase3D::Loading;
                foot.phaseSeconds = 0.0f;
                foot.targetWorld = landing;
                foot.targetOrientationWorld = flat;
            } else {
                foot.phase = FootPhase3D::TouchdownSearch;
                foot.phaseSeconds += deltaTime;
                foot.targetWorld = landing;
                foot.targetOrientationWorld = flat;
                foot.goalWorld = landing;
                foot.goalOrientationWorld = flat;
            }
        } else {
            foot.phase = FootPhase3D::Stance;
            foot.targetWorld = foot.anchorWorld;
            foot.targetOrientationWorld = foot.anchorOrientationWorld;
        }
        m_scriptedLast[side] = true;
    }

    // Agenda de apoio.
    const bool leftDown = supportsWeight(plan.feet[0].phase);
    const bool rightDown = supportsWeight(plan.feet[1].phase);
    if (leftDown && rightDown) {
        plan.schedule = SupportSchedule3D::DoubleSupport;
    } else if (leftDown) {
        plan.schedule = SupportSchedule3D::LeftSupport;
    } else if (rightDown) {
        plan.schedule = SupportSchedule3D::RightSupport;
    } else {
        plan.schedule = plan.feet[0].followingGait && plan.feet[1].followingGait
                && !physical.supported
            ? SupportSchedule3D::PlannedFlight : SupportSchedule3D::UnexpectedNoSupport;
    }
    // Agenda observada (R1): nenhum pe no chao sem voo planejado e voo
    // inesperado, mesmo que o plano diga apoio.
    const bool leftSeen = physical.feet[0].contactObserved;
    const bool rightSeen = physical.feet[1].contactObserved;
    const SupportSchedule3D before = plan.observedSchedule;
    plan.observedSchedule = leftSeen && rightSeen ? SupportSchedule3D::DoubleSupport
        : leftSeen ? SupportSchedule3D::LeftSupport
        : rightSeen ? SupportSchedule3D::RightSupport
        : plan.schedule == SupportSchedule3D::PlannedFlight ? SupportSchedule3D::PlannedFlight
        : SupportSchedule3D::UnexpectedNoSupport;
    if (plan.observedSchedule == SupportSchedule3D::UnexpectedNoSupport && (leftDown || rightDown)) {
        ++plan.unexpectedFlightTicks;
        if (before != SupportSchedule3D::UnexpectedNoSupport) ++plan.unexpectedFlightEvents;
    }
}

} // namespace MatterEngine
