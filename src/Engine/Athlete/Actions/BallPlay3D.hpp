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
//  - Plano v4: a passada nao corre mais atras do gesto - o gesto termina
//    inteiro e o pe volta ao do clipe andando com ele (BallMove3D); o corpo
//    desacelera um pouco durante o gesto; o corpo do gesto (pelve, tronco,
//    bracos e olhar) pela intensidade, com um jeito diferente a cada toque.
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
    // diferente da de entao): o movimento vai direto a volta.
    float interruptTurn = 0.6f;
    float interruptSpeed = 0.8f;
    // A perna sai do movimento nesta mistura (s): parado no pouso (sem o pe
    // do clipe) e ja junto do pe do clipe (os dois no mesmo lugar).
    float handOverSeconds = 0.12f;
    float joinedHandOverSeconds = 0.05f;
    // A velocidade pedida ao corpo durante o gesto (o toque e o chute), numa
    // mola de speedScaleSeconds (s).
    float touchSpeedScale = 0.9f;
    float shotSpeedScale = 0.85f;
    float speedScaleSeconds = 0.1f;
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
    // O corpo do gesto pela intensidade: bodyPowerFloor dele no toque mais
    // fraco, inteiro no mais forte; no corpo fisico, vezes physicalBodyShare
    // (o fisico ja reage com o corpo dele). A cada toque cada parte varia ate
    // bodyVariation (fracao) e os punhos ate wristVariation (m).
    float bodyPowerFloor = 0.35f;
    float physicalBodyShare = 0.6f;
    float bodyVariation = 0.3f;
    float wristVariation = 0.05f;
    // O polo do joelho sai do joelho da pose nesta mistura (s).
    float kneeBlendSeconds = 0.06f;
    // A velocidade dos pes (do clipe e da pose) vem filtrada neste tempo (s).
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
    [[nodiscard]] const BallMove3D& previous() const { return m_previous; }
    [[nodiscard]] bool ready() const { return m_ready; }
    // O movimento indo direto a volta (a mudanca de movimento depois da
    // forca).
    [[nodiscard]] bool interrupting() const { return m_interrupting; }

    // A bola na area em volta da pelve dada (andando a velocity, ela vai a
    // frente).
    [[nodiscard]] static bool inArea(Vec3 ball, float radius, Vec3 pelvis, const BallPlaySettings3D& settings,
        Vec3 velocity = {});

private:
    // O jeito do corpo neste toque: o tanto (pela intensidade) e quanto cada
    // parte varia.
    struct BodyStyle {
        float amount = 1.0f;
        float shift = 1.0f, drop = 1.0f, yaw = 1.0f, side = 1.0f, bend = 1.0f, arms = 1.0f, look = 1.0f;
        float pace = 1.0f;
        std::array<Vec3, 2> wrist {};
    };

    void startMove(const BallHandlingInput3D& input, const BallPlaySettings3D& settings);
    void applyForce(const BallHandlingInput3D& input, const BallOutcomeSettings3D& outcome,
        const BallPlaySettings3D& settings, BallHandlingOutput3D& output);
    void bodyTargets(const BallHandlingInput3D& input, const BallPlaySettings3D& settings,
        BallHandlingOutput3D& output);
    void settle(const BallMove3D& move, const BallPlaySettings3D& settings);
    [[nodiscard]] BallMoveLive3D live(const BallHandlingInput3D& input, int side) const;
    [[nodiscard]] float random();

    bool m_ready = false;
    AthleteLegs3D m_legs;
    AthleteFootGeometry3D m_geometry;
    std::array<float, 2> m_ankleHeight { 0.08f, 0.08f };
    ActionQueue3D m_queue;
    // O movimento em curso e o anterior ainda voltando na outra perna (um
    // toque novo com a outra perna nao corta a volta dele).
    BallMove3D m_move;
    BallMove3D m_previous;
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
    // O apoio do movimento (o outro pe).
    int m_support = -1;
    // Os pes do clipe (o tornozelo) e os da pose de fato: a velocidade deles,
    // filtrada.
    std::array<Vec3, 2> m_clipAnkle {};
    std::array<Vec3, 2> m_clipAnkleVelocity {};
    std::array<Vec3, 2> m_clipAnkleAcceleration {};
    std::array<Vec3, 2> m_poseFootPrevious {};
    std::array<Vec3, 2> m_poseFootVelocity {};
    bool m_feetKnown = false;
    // A escala da velocidade pedida ao corpo (na mola).
    float m_speedScale = 1.0f;
    // O jeito do corpo neste toque e o sorteio dele.
    BodyStyle m_style;
    std::uint32_t m_seed = 0x2545F491u;
    // O corpo: o peso e os alvos nas molas; o deslocamento e a descida da
    // pelve aplicados no tick anterior (ja estao na pose).
    bool m_bodyLive = false;
    int m_bodySide = 0;
    Vec3 m_kneeStart { 1.0f, 0.0f, 0.0f };
    Vec3 m_previousKnee { 1.0f, 0.0f, 0.0f };
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
    // A perna saindo do movimento depois do fim dele: o tornozelo, o pe e o
    // joelho do fim (junto do clipe, eles vao com o pe do clipe), o lado e o
    // que falta da mistura.
    Vec3 m_lastAnkle;
    Quaternion m_lastFoot;
    Vec3 m_lastKnee { 1.0f, 0.0f, 0.0f };
    int m_lastSide = 0;
    bool m_lastJoined = false;
    float m_settleLeft = 0.0f;
    float m_settleSpan = 0.12f;
};

} // namespace MatterEngine
