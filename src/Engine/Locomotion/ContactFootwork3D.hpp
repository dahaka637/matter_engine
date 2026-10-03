#pragma once

#include "Engine/Character/CharacterControlTypes3D.hpp"

#include <array>
#include <cstdint>
#include <functional>

namespace MatterEngine {

// Planejador unico de apoio e passos. E o unico escritor do estado oficial
// de cada pe (FootPlan3D): fase, motivo, identidade do passo, ancora, pouso.
// A animacao propoe (onde a pose quer o pe); o planejador decide; o contato
// medido (CharacterStateEstimator3D) confirma. Nao escreve transform nem
// forca: publica alvos que a locomocao leva as pernas por IK e motores.
//
// Migracao (etapa E3 do plano Astra): parado, o planejador decide e executa
// os passos (acomodacao, giro, recuperacao, refazer a base). Andando, a
// passada do clipe ainda decide os pes na locomocao; o planejador segue o
// estado dela (followingGait) para assumir sem salto quando o corpo para. Um
// passo iniciado aqui continua daqui ate o pe carregar, mesmo que a passada
// recomece.

// Fase operacional de um pe. Os fatos fisicos (tocando, apoiando) ficam no
// estimador, independentes.
enum class FootPhase3D : std::uint8_t {
    // Apoiado: o alvo e a ancora no chao.
    Stance,
    // Vai sair: o peso passa para o outro pe; o alvo ainda e a ancora.
    Unloading,
    // No ar, seguindo a trajetoria ate o pouso.
    Swing,
    // O tempo do passo acabou sem contato: desce procurando o chao. Nunca
    // vira apoio so por tempo.
    TouchdownSearch,
    // Tocou (contato medido): recebe o peso onde tocou.
    Loading
};

// Razao primaria de um passo. Aumentar a urgencia pode trocar a razao do
// mesmo passo sem reiniciar a trajetoria (a revisao sobe).
enum class StepReason3D : std::uint8_t {
    None,
    Start,
    Locomotion,
    Brake,
    Turn,
    Recovery,
    Terrain,
    Landing,
    ReestablishSupport,
    // Acomodacao: o pe apoiado ficou longe de onde a pose o quer.
    Settle
};

// Agenda de apoio do corpo inteiro neste tick.
enum class SupportSchedule3D : std::uint8_t {
    DoubleSupport,
    LeftSupport,
    RightSupport,
    // Os dois fora do chao por plano (pulo, corrida).
    PlannedFlight,
    // Nenhum pe apoiando sem ter sido planejado.
    UnexpectedNoSupport
};

// Por que o pe saiu do chao (E5, R2 do pacote v2): a razao da decisao, nao
// uma medida de carga - a engine nao mede a carga normal de cada pe.
enum class ReleaseReason3D : std::uint8_t {
    None,
    // Carga estimada abaixo de 0,30 (proxy de ZMP/alavanca, nao do solver).
    LoadEstimate,
    // Carga estimada abaixo de 0,42 depois de 0,2 s tentando.
    PartialLoadEstimate,
    // Geometria da transferencia: o DCM fez 60% do caminho ate o outro pe.
    TransferGeometry,
    // O pe que sai a frente no rumo, pela agenda do pouso previsto.
    LeadingAgenda,
    // O pe ja tinha saido do chao (contato observado sumiu).
    AirborneObserved,
    // Passo de recuperacao, pouso ou restabelecer apoio.
    Recovery,
    // O prazo acabou sem nenhuma condicao: falha explicita da transferencia.
    TransferFailed,
    Count
};

// O pe fisicamente fora do chao depois da soltura (R2): Releasing = soltou,
// mas ainda ha contato observado; Airborne = o contato sumiu.
enum class PhysicalRelease3D : std::uint8_t { Grounded, Releasing, Airborne };

// Papel de apoio que o plano espera do pe (R1) e o apoio observado.
enum class PlannedSupport3D : std::uint8_t { None, Stance, Accepting, TransferOut };
enum class ObservedSupport3D : std::uint8_t { None, Touching, Supporting, Slipping };

// Forma do caminho do pe no ar.
enum class SwingPath3D : std::uint8_t {
    // Reto e baixo, desviando do pe de apoio (recuperacao, refazer a base).
    Direct,
    // Em arco em volta do pe de apoio (acomodacao, segundo passo do giro).
    AroundSupport,
    // Girando sobre a bola do pe (abrir o giro na base lado a lado).
    Pivot
};

struct FootSurface3D {
    bool valid = false;
    float height = 0.0f;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
};

// Plano de um pe: o unico estado oficial dele. Posicoes sao a origem do
// link do pe (o que o IK recebe).
struct FootPlan3D {
    FootPhase3D phase = FootPhase3D::Stance;
    StepReason3D reason = StepReason3D::None;
    SwingPath3D path = SwingPath3D::Direct;
    // Identidade do passo (sobe a cada passo novo; 0 = nenhum ainda) e
    // revisao (sobe quando o mesmo passo muda de razao ou de ritmo).
    std::uint32_t stepId = 0;
    std::uint32_t revision = 0;
    // A passada do clipe decide este pe agora (o plano so a acompanha).
    bool followingGait = true;
    // Apoio: onde o pe esta preso e o chao dele.
    Vec3 anchorWorld;
    Quaternion anchorOrientationWorld;
    FootSurface3D surface;
    // Passo: de onde saiu (medido na soltura), para onde vai e o ritmo.
    Vec3 liftoffWorld;
    Quaternion liftoffOrientationWorld;
    float liftoffGroundHeight = 0.0f;
    Vec3 goalWorld;
    Quaternion goalOrientationWorld;
    FootSurface3D goalSurface;
    float phaseSeconds = 0.0f;
    float swingSeconds = 0.0f;
    float swingDurationSeconds = 0.0f;
    float urgency = 0.0f;
    // Giro: rumo da pelve quando o passo comecou.
    float turnFromFacing = 0.0f;
    // Folga extra do arco depois de esbarrar em algo no caminho (m).
    float extraClearance = 0.0f;
    // Alvo deste tick para a perna.
    Vec3 targetWorld;
    Quaternion targetOrientationWorld;
    // Diagnostico do ultimo passo: saiu descarregado (ou por emergencia/
    // tempo), como pousou.
    bool releasedUnloaded = false;
    // Por que saiu (R2) e se ja esta no ar de fato.
    ReleaseReason3D releaseReason = ReleaseReason3D::None;
    PhysicalRelease3D physicalRelease = PhysicalRelease3D::Grounded;
    // Papel de apoio planejado e apoio observado neste tick (R1).
    PlannedSupport3D plannedSupport = PlannedSupport3D::None;
    ObservedSupport3D observedSupport = ObservedSupport3D::None;
    bool touchdownEarly = false;
    // O pe nunca saiu do chao no ultimo passo (foi arrastado ate o pouso).
    bool dragged = false;
    ContactEvidence3D touchdownEvidence = ContactEvidence3D::None;
};

// O que a referencia de movimento propoe para um pe neste tick: onde a pose
// (sob o corpo, agora) quer o pe. So proposta.
struct FootReference3D {
    Vec3 poseWorld;
    Quaternion poseOrientationWorld;
};

// Estado de um pe na passada do clipe (enquanto ela ainda decide o pe).
struct GaitFootState3D {
    bool planted = false;
    Vec3 lockedWorld;
    Quaternion lockedOrientationWorld;
    FootSurface3D surface;
};

struct ContactFootworkInput3D {
    const CharacterPhysicalState3D* physical = nullptr;
    const RagdollState3D* state = nullptr;
    // O corpo esta parado em pe: o planejador decide os passos. Fora disso,
    // a passada do clipe decide (salvo passos daqui ja em andamento).
    bool standing = false;
    // Pes podem dar passos (nao caindo sem volta, nem levantando, no chao).
    bool allowSteps = false;
    // Passos de recuperacao liberados (modo fisico, recem-levantado ha mais
    // de 0,8 s, sem queda comprometida).
    bool allowRecovery = false;
    // Refazer a base a partir do corpo (depois de levantar): pes rentes ao
    // chao viram apoio onde estao, os outros dao um passo ate a base.
    bool seedFromBody = false;
    float facingYawRadians = 0.0f;
    // Giro parado: para onde a pelve vai.
    float standingTurnTargetYawRadians = 0.0f;
    Quaternion heading;
    // Raiz da pose de referencia (mundo): o giro gira a pose em volta dela e
    // o peso se desloca a partir dela.
    Vec3 referenceRootWorld;
    std::array<FootReference3D, 2> reference {};
    // Ponto de captura usado para planejar (velocidade do COM filtrada e o
    // deslocamento da forca externa sustentada), quanto sai do apoio medido
    // e para onde; velocidade do COM usada no mesmo calculo.
    Vec3 capturePointWorld;
    float captureOutsideMeters = 0.0f;
    Vec3 captureDirectionWorld;
    Vec3 planningComVelocityWorld;
    // O jogador pede movimento, e a velocidade pedida (no plano).
    bool locomoting = false;
    Vec3 desiredVelocityWorld;
    // A perna nao alcanca a ancora deste pe (o IK ficou longe no tick
    // anterior ou o quadril esta alem do comprimento da perna).
    std::array<bool, 2> anchorUnreachable {};
    std::array<GaitFootState3D, 2> gait {};
    // Chao sob um ponto (origem do link do pe), com a ponta para `forward`.
    std::function<FootSurface3D(std::size_t side, Vec3 atWorld, Vec3 forwardWorld)> surfaceAt;
    // Quanto mover um pouso ao longo do pe para ele caber inteiro num nivel
    // (quina de degrau); zero se ja cabe ou se nao ha terreno para consultar.
    std::function<Vec3(Vec3 atWorld, Vec3 forwardWorld)> fitOnLevel;
};

struct CharacterContactPlan3D {
    bool valid = false;
    std::array<FootPlan3D, 2> feet {};
    SupportSchedule3D schedule = SupportSchedule3D::DoubleSupport;
    // A agenda observada pelo contato estrito (R1): o solver decide se ha
    // apoio; o plano so diz qual deseja.
    SupportSchedule3D observedSchedule = SupportSchedule3D::DoubleSupport;
    // Deslocamento pedido a pelve (mundo) para o peso ir ao pe de apoio
    // durante um passo de acomodacao/giro. O controle das pernas o produz.
    Vec3 supportShiftWorld;
    // Recuperacao: ponto de captura fora do apoio PLANEJADO (m), urgencia e
    // o pe escolhido neste tick (-1 nenhum); recuperando = precisou ha menos
    // de 0,45 s.
    float plannedOutsideMeters = 0.0f;
    float recoveryUrgency = 0.0f;
    int recoverySide = -1;
    bool recovering = false;
    // Carga de cada pe (fracao do total medido nos dois, filtrada).
    std::array<float, 2> loadShare { 0.5f, 0.5f };
    // Transferencia de apoio em curso (E5, fase 4): o pe que descarrega, o
    // que recebe, quanto do caminho ate o apoio dele o centro de massa deve
    // ter feito (0 a 0,85) e esse ponto de apoio. UMA fonte para quem move o
    // corpo (referencia corporal) e para a antecipacao (esforco de contato).
    struct SupportTransfer3D {
        int outgoingFoot = -1;
        int incomingFoot = -1;
        float progress = 0.0f;
        Vec3 supportPointWorld;
    } supportTransfer;
    // Contadores acumulados desde o reset (diagnostico e testes).
    std::uint32_t stepsStarted = 0;
    std::uint32_t touchdownsByContact = 0;
    std::uint32_t earlyTouchdowns = 0;
    std::uint32_t touchdownSearches = 0;
    std::uint32_t forcedReleases = 0;
    // Transicao que "passou" pelo prazo, nao pelo evento fisico (E5, fases
    // 5-6): passo de marcha solto com o pe ainda carregado, e carga que virou
    // apoio pelo tempo sem o pe ter recebido o peso.
    std::uint32_t locomotionForcedReleases = 0;
    // (dos soltos carregados: pela regra de descarga parcial - carga < 0,42
    // depois de 0,2 s - e pelo prazo)
    std::uint32_t locomotionPartialReleases = 0;
    std::uint32_t locomotionTimeoutReleases = 0;
    std::uint32_t loadingTimeouts = 0;
    // Carga que nao foi aceita ate o teto (E5): vira apoio com falha
    // explicita.
    std::uint32_t loadAcceptanceFailures = 0;
    // Balanco da marcha (E5, fase 10): passos em que a revisao do alvo foi
    // limitada (35-60% do balanco) e em que foi recusada (depois de 60%,
    // o pouso pedido a mais de 3 cm do comprometido).
    // Passos de marcha em que o pe que saia a frente do outro no rumo (de
    // lado, o que abre) saiu pela agenda do pouso previsto (fase 11), com a
    // carga que tivesse.
    std::uint32_t locomotionLeadingReleases = 0;
    // Chamadas do pouso previsto (fase 11; com o seletor desligado, zero).
    std::uint32_t placementPredictorCalls = 0;
    // Giro parado (diagnostico): rumo da base dos pes, torcao da pelve contra
    // ela e o giro que falta ate o alvo (rad), neste tick.
    float standingStanceYaw = 0.0f;
    float standingTwist = 0.0f;
    float standingTurnRemaining = 0.0f;
    // Apoio observado contra o planejado (R1): ticks de apoio ficticio por
    // papel (apoio, aceitando, saindo), eventos de apoio perdido, e ticks e
    // eventos de voo inesperado (andando, nenhum pe no chao).
    std::array<std::uint32_t, 3> fakeSupportTicks {};
    std::uint32_t supportLostEvents = 0;
    std::uint32_t unexpectedFlightTicks = 0;
    std::uint32_t unexpectedFlightEvents = 0;
    // Solturas por razao (R2; indice = ReleaseReason3D).
    std::array<std::uint32_t, static_cast<std::size_t>(ReleaseReason3D::Count)> releasesByReason {};
    std::uint32_t swingRevisionsBounded = 0;
    std::uint32_t swingRevisionsRefused = 0;
    // Passos em que o pe nunca saiu do chao (arrastado: a carga nao passou),
    // e passos comuns desistidos na descarga (a carga nao passou em 0,4 s).
    std::uint32_t draggedSteps = 0;
    std::uint32_t abortedSteps = 0;
    std::uint32_t blockedSwings = 0;
    std::uint32_t reasonChanges = 0;
    // Maior tempo procurando o chao num pouso (s).
    float longestTouchdownSearchSeconds = 0.0f;
};

struct ContactFootworkSettings3D {
    // Passo de recuperacao mira o ponto de captura previsto no pouso (senao,
    // a pose sob o corpo). Ligado quando as pernas carregam o equilibrio
    // (E4): sem a ajuda na pelve, mirar a pose sob o corpo deixava ele passar
    // do pe (varredura: 80 quedas em 216 contra 34). Com a ajuda, piora
    // (31 contra 9): a ajuda ja freia o corpo.
    bool recoveryCaptureTargeting = false;
    // E5 (experimento): andando, o planejador tambem decide os passos (em vez
    // de seguir a passada do clipe) - o pe pousa onde o corpo mantem a
    // velocidade pedida.
    bool locomotionStepping = false;
    // E5 (pacote do GPT, fase 11, A/B): o pouso da marcha escolhido entre
    // candidatos em volta do nominal pelo que o corpo faz nos dois passos
    // seguintes (pendulo invertido na agenda medida). Desligado, o nominal.
    bool locomotionPlacementPredictor = false;
    // E5 (R0 do pacote v2, integridade dos A/B): as mudancas da marcha pelo
    // planejador que valem tambem sem a referencia corporal, isoladas.
    // Ligadas = o comportamento atual.
    // Carga aceita pelo evento (fase 5): apoiado 25 ms sem escorregar, e a
    // marcha so descarrega o outro pe depois. Desligada: 50 ms de prazo.
    bool loadAcceptanceByEvent = true;
    // Perna que pousou 25% mais mole na aceitacao (fase 9).
    bool acceptanceCompliance = true;
    // O pe de tras sai com o DCM a 60% do caminho ate o outro (H18).
    bool transferGeometryRelease = true;
    // Carga estimada em dupla sustentacao pelo ZMP da aceleracao observada
    // (desligada: pela alavanca do COM entre as solas).
    bool zmpLoadEstimate = true;
    // Bancada C (diagnostico do harness): parado, descarrega este pe (0/1)
    // sem solta-lo - a transferencia de apoio isolada. -1 desliga.
    int benchTransferFoot = -1;
};

class ContactFootwork3D {
public:
    void reset(const RagdollProfile3D& profile);
    void update(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        float deltaTime);
    [[nodiscard]] const CharacterContactPlan3D& plan() const { return m_plan; }
    [[nodiscard]] ContactFootworkSettings3D& settings() { return m_settings; }
    // O planejador decide este pe agora (parado, ou um passo daqui em
    // andamento).
    [[nodiscard]] bool owns(std::size_t side) const {
        return side < 2 && !m_plan.feet[side].followingGait;
    }

private:
    void startStep(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, StepReason3D reason, SwingPath3D path, float urgency);
    void makeRecovery(const ContactFootworkInput3D& input, std::size_t side, float urgency);
    // Pe sem apoio decidido (nascer, assumir da passada, fim do levantar):
    // rente ao chao vira apoio onde esta; erguido, um passo ate a base.
    void seedFoot(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side);
    void release(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, ReleaseReason3D reason);
    void land(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side);
    void executeSwing(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, float deltaTime);
    // Onde o passo deste pe quer pousar agora (origem do link, orientacao
    // plana do pouso ainda sem o chao).
    Vec3 wantedLanding(const ContactFootworkInput3D& input, std::size_t side,
        Quaternion& orientation) const;
    Vec3 locomotionLanding(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, Vec3 pose, Quaternion orientation) const;
    Vec3 predictedLocomotionLanding(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, Vec3 pose, Quaternion orientation, const Vec3* committed) const;
    Vec3 captureLanding(const RagdollProfile3D& profile, const ContactFootworkInput3D& input,
        std::size_t side, Vec3 pose, Quaternion orientation) const;
    float stanceYaw(const RagdollProfile3D& profile, const ContactFootworkInput3D& input) const;

    ContactFootworkSettings3D m_settings;
    CharacterContactPlan3D m_plan;
    std::array<std::size_t, 2> m_feet {};
    std::size_t m_linkCount = 0;
    std::uint32_t m_nextStepId = 1;
    // Giro parado: o pe que ainda deve o segundo passo (-1 nenhum).
    int m_pendingTurnFoot = -1;
    float m_sinceRecoveryNeeded = 10.0f;
    // Recuperacao: onde o pe pousa (acompanha a pose ate 60% do passo).
    std::array<Vec3, 2> m_recoveryTarget {};
    // Encaixe do pouso num degrau: para qual pouso foi medido e o
    // deslocamento (refeito so quando o pouso anda mais de 3 cm).
    std::array<Vec3, 2> m_fitCenter {};
    std::array<Vec3, 2> m_fitShift {};
    std::array<bool, 2> m_fitValid {};
    // Giro: o sentido em que o passo de cada pe comecou (+1 esquerda).
    std::array<float, 2> m_turnSign { 1.0f, 1.0f };
    // Ultimo pe que pousou um passo de recuperacao e ha quanto tempo (s).
    int m_lastRecoverySide = -1;
    float m_sinceRecoveryLanding = 10.0f;
    // Locomocao pelo planejador (E5): o ultimo pe que deu passo e ha quanto
    // tempo ele pousou.
    int m_lastLocomotionSide = -1;
    float m_sinceLocomotionTouchdown = 10.0f;
    // Intervalo medido entre pousos de marcha (s), filtrado: o T do modelo.
    float m_locomotionPeriod = 0.45f;
    // Bancada C: o pe que a bancada esta segurando na descarga.
    std::array<bool, 2> m_benchHolding {};
    // Apoio perdido no tick anterior, por pe (R1: evento na borda).
    std::array<bool, 2> m_supportLost {};
    // Balanco da marcha (E5, fase 10): a referencia do pe no plano como
    // estado (posicao, velocidade, aceleracao), refeita a cada tick como
    // quintica ate o alvo comprometido - revisar o alvo muda o resto da
    // curva, nunca o ponto de agora.
    struct SwingTrajectory3D {
        bool valid = false;
        std::uint32_t stepId = 0;
        Vec3 position;
        Vec3 velocity;
        Vec3 acceleration;
        Vec3 committed;
        // O alvo em 35% do balanco: a revisao limitada anda em volta dele.
        Vec3 boundedFrom;
        bool boundedFromValid = false;
        bool boundedCounted = false;
        bool refusedCounted = false;
    };
    std::array<SwingTrajectory3D, 2> m_swingTrajectory {};
    // Deslocamento de peso pedido quando o pe soltou (fracao do caminho ate o
    // pe de apoio e o limite dela), mantido no balanco.
    std::array<float, 2> m_releaseShiftFraction {};
    std::array<float, 2> m_releaseShiftLimit {};
    // Espera antes do proximo passo comum deste pe (s), e a espera que cresce
    // a cada passo arrastado seguido.
    std::array<float, 2> m_stepCooldown {};
    std::array<float, 2> m_draggedBackoff {};
    // No balanco: ticks seguidos sem contato de apoio e se o pe ja saiu do
    // chao neste passo (so assim um contato cedo e pouso).
    std::array<int, 2> m_ticksWithoutContact {};
    std::array<bool, 2> m_liftedOff {};
    // Ultimo deslocamento de peso pedido: sai suave quando nada mais o pede.
    Vec3 m_residualShift;
    bool m_initialized = false;
};

} // namespace MatterEngine
