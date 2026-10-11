#pragma once

#include "Engine/Athlete/Actions/BallHandlingIO3D.hpp"
#include "Engine/Athlete/Actions/BallMove3D.hpp"
#include "Engine/Athlete/Core/CriticalSpring3D.hpp"
#include "Engine/Athlete/Core/FootGeometry3D.hpp"
#include "Engine/Athlete/Intent/ActionQueue3D.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// O manejo (plano v3, pedidos do usuario de 10/10 noite): a acao depende so
// de a bola estar na area do jogador; a forca sai na hora do aceite; o gesto
// e um movimento padronizado (BallMove3D) - o pe nunca procura a bola.
//  - Area (a da V3): a base da bola ate a cintura e, no plano, ate uma perna
//    esticada da pelve (andando, tambem um pouco a frente).
//  - Pedido guardado ate requestBuffer: a bola entrando na area nesse tempo,
//    ele sai; com a bola na area, sai sempre (a garantia).
//  - O movimento: o da situacao (o rumo do toque contra a frente do corpo; a
//    perna daquele lado), no referencial do corpo.
//  - Andando, o movimento entra na passada: o ritmo do clipe acelera ou
//    freia (gaitRateScale) para o pe do toque pousar no clipe quando o
//    movimento pousa; com a perna do toque apoiada e a outra no ar, a outra
//    pousa ja (o pouso rapido das pegadas) e fica de apoio.
struct BallPlaySettings3D {
    // A area: a base da bola ate isto acima da pelve (m) e, no plano, ate
    // areaRadius dela (m). Andando, o centro dela vai a frente no rumo do
    // movimento: ate areaLead (m), cheio a areaLeadSpeed (m/s).
    float areaRadius = 0.95f;
    float waistAbovePelvis = 0.05f;
    float areaLead = 0.45f;
    float areaLeadSpeed = 4.5f;
    // O pedido guardado (s); a bola que esteve na area ate guaranteeGrace
    // atras ainda vale (s); o intervalo depois de cada forca (s).
    float requestBuffer = 0.25f;
    float guaranteeGrace = 0.06f;
    float cooldown = 0.25f;
    // A velocidade "a frente" do toque: a do corpo daqui a carryAhead (da aba
    // "Forca na bola"), indo a pedida nesta aceleracao (m/s2) - a do manejo
    // V3, onde a forca foi calibrada.
    float carryAcceleration = 16.0f;
    // Depois da forca, o movimento pedido mudando (o rumo mais que
    // interruptTurn rad, ou a velocidade pedida mais que interruptSpeed m/s
    // diferente da de entao): o movimento vai direto ao pouso.
    float interruptTurn = 0.6f;
    float interruptSpeed = 0.8f;
    // O ritmo da passada durante o movimento: entre estes fatores, numa mola
    // de gaitRateSeconds (s); depois do pouso do movimento, ainda ate
    // gaitTailSeconds (s) esperando o pouso do clipe.
    // (Nunca mais devagar que o normal: freando a passada, a outra perna
    // ficava no ar e o corpo flutuava parado na pose.)
    float gaitRateMinimum = 1.0f;
    float gaitRateMaximum = 2.0f;
    float gaitRateSeconds = 0.08f;
    float gaitTailSeconds = 0.3f;
    // Andando, o movimento dura o tempo ate o pouso do pe dele no clipe,
    // entre estes (s).
    float moveMinimumSeconds = 0.18f;
    float moveMaximumSeconds = 0.30f;
    // Sem as pegadas, a perna sai do movimento nesta mistura depois do pouso
    // (s); com elas, as pegadas a prendem onde pousou, sem mistura.
    float handOverSeconds = 0.12f;
    float handOverStepsSeconds = 0.08f;
    // O corpo: o preparo, os alvos, o olhar e os bracos nestas molas (s); a
    // pelve sobre o apoio ate maximumPelvisShift (m); andando, o corpo do
    // gesto pesa (1 - runningBodyShare x andando).
    float prepareSeconds = 0.12f;
    float targetSeconds = 0.13f;
    float lookSeconds = 0.20f;
    float armSeconds = 0.15f;
    float maximumPelvisShift = 0.10f;
    float pelvisMassShare = 0.85f;
    float runningBodyShare = 0.85f;
    // O polo do joelho sai do joelho da pose nesta mistura (s).
    float kneeBlendSeconds = 0.06f;
    // A velocidade do pe do estilo vem filtrada neste tempo (s).
    float styleVelocitySeconds = 0.03f;
    BallMoveSettings3D move;
};

class BallPlay3D {
public:
    void reset(const RagdollProfile3D& profile);
    void update(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallPlaySettings3D& settings, BallHandlingOutput3D& output);
    // A bola do pedido ou da acao em andamento (0 nenhuma).
    [[nodiscard]] std::uint64_t targetBallKey() const;
    [[nodiscard]] const ActionQueue3D& queue() const { return m_queue; }
    [[nodiscard]] const BallMove3D& move() const { return m_move; }
    [[nodiscard]] bool ready() const { return m_ready; }
    // O movimento indo direto ao pouso (a mudanca de movimento depois da
    // forca) e o ritmo da passada pedido agora.
    [[nodiscard]] bool interrupting() const { return m_interrupting; }
    [[nodiscard]] float gaitRateScale() const { return m_gaitRate; }

    // A bola na area em volta da pelve dada (andando a velocity, ela vai a
    // frente).
    [[nodiscard]] static bool inArea(Vec3 ball, float radius, Vec3 pelvis, const BallPlaySettings3D& settings,
        Vec3 velocity = {});

private:
    void startMove(const BallHandlingInput3D& input, const BallPlaySettings3D& settings);
    void applyForce(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallPlaySettings3D& settings, BallHandlingOutput3D& output);
    void bodyTargets(const BallHandlingInput3D& input, const BallPlaySettings3D& settings,
        BallHandlingOutput3D& output);
    [[nodiscard]] BallMoveLive3D live(const BallHandlingInput3D& input, int side) const;

    bool m_ready = false;
    AthleteLegs3D m_legs;
    AthleteFootGeometry3D m_geometry;
    std::array<float, 2> m_ankleHeight { 0.08f, 0.08f };
    ActionQueue3D m_queue;
    BallMove3D m_move;
    // O pedido aceito e a bola dele (o movimento em curso).
    BallOutcomeRequest3D m_intent;
    std::uint64_t m_ballKey = 0;
    // A garantia: ha quanto tempo a bola esteve na area (s); o intervalo
    // depois da forca (s).
    float m_sinceInArea = 10.0f;
    float m_cooldown = 0.0f;
    // O tempo que o pedido guardado esperou o movimento anterior.
    float m_busyWait = 0.0f;
    std::uint32_t m_busyRecord = 0;
    // A mudanca de movimento depois da forca.
    Vec3 m_requestedAtAccept;
    bool m_interrupting = false;
    // O ritmo da passada: o fator, se o clipe ja levou o pe do toque ao
    // balanco (e se ja o pousou), o tempo depois do pouso do movimento.
    float m_gaitRate = 1.0f;
    float m_gaitRateWanted = 1.0f;
    bool m_gaitCoupled = false;
    bool m_clipSwung = false;
    float m_tail = 0.0f;
    // O apoio do movimento (o outro pe) e se ele pousa rapido.
    int m_support = -1;
    bool m_quickPlant = false;
    // Os pes do estilo (a velocidade deles, filtrada).
    std::array<Vec3, 2> m_styleFootPrevious {};
    std::array<Vec3, 2> m_styleFootVelocity {};
    bool m_styleFootKnown = false;
    // O corpo: o peso e os alvos nas molas; o deslocamento e a descida da
    // pelve aplicados no tick anterior (ja estao na pose).
    bool m_bodyLive = false;
    int m_bodySide = 0;
    Vec3 m_kneeStart { 1.0f, 0.0f, 0.0f };
    SmoothSpring3D<float> m_weight;
    SmoothSpring3D<Vec3> m_shift;
    SmoothSpring3D<float> m_drop;
    SmoothSpring3D<float> m_yaw;
    SmoothSpring3D<float> m_trunkSide;
    SmoothSpring3D<float> m_trunkBend;
    SmoothSpring3D<float> m_armSwing;
    std::array<SmoothSpring3D<Vec3>, 2> m_wrist {};
    SmoothSpring3D<Vec3> m_look;
    SmoothSpring3D<float> m_lookWeight;
    Vec3 m_appliedShift;
    float m_appliedDrop = 0.0f;
    // Sem as pegadas: a perna saindo do movimento depois do pouso (o
    // tornozelo, o pe e o joelho do pouso, o que falta da mistura).
    Vec3 m_lastAnkle;
    Quaternion m_lastFoot;
    Vec3 m_lastKnee { 1.0f, 0.0f, 0.0f };
    int m_lastSide = 0;
    float m_settleLeft = 0.0f;
    float m_settleSpan = 0.12f;
};

} // namespace MatterEngine
